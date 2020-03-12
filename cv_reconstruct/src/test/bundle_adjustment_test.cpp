//
// bundle_adjustment_test.cpp
// Test out bundle adjustment techniques using G2o
//

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "pipeline/StereoFrame.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "reconstruct/Localizer.hpp"

#include "tests_common.hpp"

#define LOCALIZATION_DATA_FILE "pipeline_localization_data.txt"

typedef g2o::BlockSolver_6_3 BlockSolver;

// Switch between LevenbergMQ or GN algorithm by commenting out the other

// Levenberg
typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg OptimizationAlgorithm;

// GN
//typedef g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType> LinearSolver;
//typedef g2o::OptimizationAlgorithmGaussNewton OptimizationAlgorithm;

#define BA_NUM_ITERATIONS 4

// Represents the frame for BA - pose and map 3D points as vertices
struct KeyFrame
{
    KeyFrame(unsigned int index, const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
        Index = index;
        Pose = pose;
        MapPoints = pcl::PointCloud<pcl::PointXYZRGB>::Ptr { new pcl::PointCloud<pcl::PointXYZRGB>() };
        pcl::copyPointCloud(*points, *MapPoints);
    }

    unsigned int Index { 0 };
    Eigen::Matrix4f Pose;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapPoints;
};

// BA on the point cloud
void BundleAdjust(std::vector<KeyFrame> frames, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result, const Camera::Calib::StereoCalib& calib);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFramePointCloud(const Pipeline::StereoFrame& frame, const Reconstruct::Reconstruct3D& reconstruct3D);

int main(int argc, char** argv)
{
    // read in localization data and convert to frames
    std::vector<Pipeline::StereoFrame> frames;
    std::vector<LocalizationData> localizationData;

    ReadLocalizationData(LOCALIZATION_DATA_FILE, localizationData);
    ConvertLocalizationsToStereoFrames(localizationData, frames);

    // get calib and config
    Camera::Calib::StereoCalib stereoCalib;
    Config::Config config;
    GetCalibAndConfig(stereoCalib, config);

    // create all the required components
    Reconstruct::Reconstruct3D reconstruct3D(stereoCalib, config);
    Reconstruct::Localizer localizer;

    // prepare clouds and key frames
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };

    std::vector<KeyFrame> keyframes;

    for (unsigned int i = 0; i < frames.size(); i++)
    {
        frames[i].ID = i;

        // get point cloud for this frame
        pc = GetFramePointCloud(frames[i], reconstruct3D);

        // get the pose for the keyframe
        Eigen::Matrix4f pose = localizer.GetFrameWorldPose(frames[i]);
        std::cout << "\nPose for frame #" << i << "\n" << pose << std::endl;

        // add camera debug shape
        //AddCameraDebugShape(*pc, (i == 0 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 ? 255 : 0));

        // update the points with a rigid-body transform
        pcl::transformPointCloud(*pc, *transformed, pose);

        // add color
        /*
        for (auto& p : *transformed)
        {
            p.rgb = 0;
            switch (i)
            {
                case 0:
                    p.r = 255;
                    break;
                case 1:
                    p.g = 255;
                    break;
                case 2:
                    p.b = 255;
                    break;
            }
        }
        */

        // create camera plane debug point cloud to see transforms
        AddCameraDebugShape(*debugCloud, (i == 0 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 ? 255 : 0));
        pcl::transformPointCloud(*debugCloud, *debugCloud, pose);

        // append to scene cloud
        *scene += *transformed;
        *scene += *debugCloud;

        // create keyframe for SLAM
        keyframes.push_back(std::move(KeyFrame(i, pose, transformed)));

        transformed->clear();
        debugCloud->clear();

        pc->clear();
    }

    // save unoptimised cloud for comparison
    pcl::io::savePCDFileBinary("ba_not_optimised.pcd", *scene);

    std::cout << "\nScene cloud prepared. Performing BA and map optimization";

    // now perform BA on this point cloud and pose
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr optimised { new pcl::PointCloud<pcl::PointXYZRGB>() };
    BundleAdjust(keyframes, optimised, stereoCalib);

    // save to disk
    std::cout << "\nSaving PCD to disk" << std::endl;
    pcl::io::savePCDFileBinary("ba_optimised.pcd", *optimised);

    std::cout << std::endl;
    return 0;
}

// Bundle Adjustment using G2O framework
void BundleAdjust(std::vector<KeyFrame> frames, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result, const Camera::Calib::StereoCalib& calib)
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // setup block solver and optimisation algorithm (for cholesky sparse decomposition)
    LinearSolver * linearSolver { new LinearSolver() };
    BlockSolver* solver { new BlockSolver(linearSolver) };
    g2o::OptimizationAlgorithm* algorithm { new OptimizationAlgorithm(solver) };

    optimizer.setAlgorithm(algorithm);

    // setup camera intrinsics and baseline
    double f = (calib.LeftCameraCalib.K(0, 0) + calib.LeftCameraCalib.K(1, 1)) / 2.0;
    Eigen::Vector2d c { calib.LeftCameraCalib.K(0, 2), calib.LeftCameraCalib.K(1, 2) };
    double b = calib.T(0);

    g2o::CameraParameters* cameraParams { new g2o::CameraParameters(f, c, 0.0) };
    cameraParams->setId(0);
    if (!optimizer.addParameter(cameraParams)) {
        std::cout << "\nFailed to add camera params to optimiser!" << std::endl;
    }

    // add poses and points (vertices to graph)
    int vertexIndex = 0;
    int edgeIndex = 0;

    // last pose to connect
    g2o::VertexSE3Expmap* previousPoseVertex { nullptr };

    // this will map the PCL point cloud index to the address of the map point vertex in the optimised graph
    std::vector<std::vector<g2o::VertexSBAPointXYZ*>> correctedPoints;

    for (const auto& frame : frames)
    {
        // add pose for this frame
        Eigen::Quaterniond q;
        q.setIdentity();

        Eigen::Vector3d t = frame.Pose.block(0, 3, 3, 1).cast<double>();
        g2o::SE3Quat pose(q, t);

        g2o::VertexSE3Expmap* poseVertex = new g2o::VertexSE3Expmap;
        poseVertex->setId(vertexIndex);
        if (frame.Index < 1) {
            poseVertex->setFixed(true);
        }
        poseVertex->setEstimate(pose);

        optimizer.addVertex(poseVertex);
        vertexIndex++;

        correctedPoints.push_back(std::vector<g2o::VertexSBAPointXYZ*>());

        // add 3D points for this frame
        for (const auto& p : frame.MapPoints->points)
        {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ;

            point->setId(vertexIndex);
            point->setMarginalized(true);
            point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));

            optimizer.addVertex(point);
            correctedPoints[frame.Index].push_back(point);
            vertexIndex++;

            // add edge connecting this 3D point to the pose vertex
            Eigen::Vector2d z = cameraParams->cam_map(pose.map(Eigen::Vector3d(p.x, p.y, p.z)));

            g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(poseVertex));
            e->setMeasurement(z);
            e->setId(edgeIndex);
            e->information() = Eigen::Matrix2d::Identity();
            e->setParameterId(0, 0);

            if (!optimizer.addEdge(e)) {
                assert(false);
            }
        }

        // connect this pose to previous camera pose
        if (previousPoseVertex != nullptr)
        {
            g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap;

            Eigen::Vector3f t1 = frame.Pose.block(0, 3, 3, 1);
            Eigen::Vector3f t2 = frames[frame.Index - 1].Pose.block(0, 3, 3, 1);
            Eigen::Vector3d t = (t2 - t1).cast<double>();

            Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
            P.block(0, 3, 3, 1) = t;

            Eigen::Quaterniond q;
            q.setIdentity();
            g2o::SE3Quat m(q, t);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(previousPoseVertex));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(poseVertex));
            e->setMeasurement(m);

            optimizer.addEdge(e);
        }

        previousPoseVertex = poseVertex;
    }

    std::cout << "\nAll edges and vertices added. Performing map optimisation" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(BA_NUM_ITERATIONS);

    // adjust all points in the point clouds for each frame
    g2o::VertexSBAPointXYZ* p { nullptr };
    pcl::PointXYZRGB point;

    for (unsigned int i = 0; i < frames.size(); i++)
    {
        for (int j = 0; j < frames[i].MapPoints->points.size(); j++)
        {
            p = correctedPoints[i][j];
            point = frames[i].MapPoints->points[j];

            point.x = p->estimate().x();
            point.y = p->estimate().y();
            point.z = p->estimate().z();

            result->push_back(point);
        }
    }

    std::cout << "\nDone. BA complete" << std::endl;
}



// Get point cloud for the given stereo frame
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFramePointCloud(const Pipeline::StereoFrame& frame, const Reconstruct::Reconstruct3D& reconstruct3D)
{
    cv::Mat disparity = reconstruct3D.GenerateDisparityMap(frame.LeftImage, frame.RightImage);

    pcl::PointCloud<pcl::PointXYZRGB> cloud = reconstruct3D.Triangulate3D(disparity, frame.LeftImage, frame.RightImage);
    //pcl::PointCloud<pcl::PointXYZRGB> cloud = reconstruct3D.GeneratePointCloud(disparity, frame.LeftImage);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc { new pcl::PointCloud<pcl::PointXYZRGB>(std::move(cloud)) };

    return pc;
}