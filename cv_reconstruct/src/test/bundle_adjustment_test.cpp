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

#include "tests_common.hpp"

#define LOCALIZATION_DATA_FILE "pipeline_localization_data.txt"
#define EARTH_RADIUS_METERS 6378137.0f

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
Eigen::Matrix4f GetFrameWorldPose(const Pipeline::StereoFrame& frame);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFramePointCloud(const Pipeline::StereoFrame& frame, const Reconstruct::Reconstruct3D& reconstruct3D);
Eigen::Vector3f ProjectGPSToMercator(float latitude, float longitude, float altitude);

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

    // prepare clouds and key frames
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene { new pcl::PointCloud<pcl::PointXYZRGB>() };
    std::vector<KeyFrame> keyframes;

    for (unsigned int i = 0; i < frames.size(); i++)
    {
        frames[i].ID = i;

        // get point cloud for this frame
        pc = GetFramePointCloud(frames[i], reconstruct3D);

        // get the pose for the keyframe
        Eigen::Matrix4f pose = GetFrameWorldPose(frames[i]);
        std::cout << "\nPose for frame #" << i << "\n" << pose << std::endl;

        // update the points with a rigid-body transform
        pcl::transformPointCloud(*pc, *pc, pose);

        // append to scene cloud
        *scene += *pc;

        // create keyframe for SLAM
        keyframes.push_back(std::move(KeyFrame(i, pose, pc)));

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
    optimizer.setVerbose(false);

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
        if (frame.Index < 2) {
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

            optimizer.addEdge(e);
        }
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

// Get world pose for the stereo frame in euclidean coordinate space
Eigen::Matrix4f GetFrameWorldPose(const Pipeline::StereoFrame& frame)
{
    // get mercator projection
    Eigen::Vector3f t = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2));

    // store first transform
    static std::unique_ptr<Eigen::Matrix4f> T0 { nullptr };
    if (T0 == nullptr)
    {
        T0 = std::make_unique<Eigen::Matrix4f>(Eigen::Matrix4f::Identity());
        T0->block(0, 0, 3, 3) = frame.Rotation;
        T0->block(0, 3, 3, 1) = t;
    }

    // this transform
    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    T1.block(0, 0, 3, 3) = frame.Rotation;
    T1.block(0, 3, 3, 1) = t;

    // convert frame to coordinate system relative to origin (world space)
    Eigen::Matrix4f T1_W = T0->inverse() * T1;

    return T1_W;
}

// Get point cloud for the given stereo frame
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFramePointCloud(const Pipeline::StereoFrame& frame, const Reconstruct::Reconstruct3D& reconstruct3D)
{
    cv::Mat disparity = reconstruct3D.GenerateDisparityMap(frame.LeftImage, frame.RightImage);

    pcl::PointCloud<pcl::PointXYZRGB> cloud = reconstruct3D.GeneratePointCloud(disparity, frame.LeftImage);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc { new pcl::PointCloud<pcl::PointXYZRGB>(std::move(cloud)) };

    return pc;
}

// GPS to Mercator Projection
// X: Right, Y: Up, Z: Forward
Eigen::Vector3f ProjectGPSToMercator(float latitude, float longitude, float altitude)
{
    // record the mercator scale
    static float mercatorScale = -1.0;
    if (mercatorScale < 0.0) {
        mercatorScale = cosf((latitude * static_cast<float>(M_PI)) / 180.0f);
    }

    Eigen::Vector3f T = Eigen::Vector3f::Zero();

    float fwd = mercatorScale * EARTH_RADIUS_METERS * ((M_PI * longitude) / 180.0);
    float left = mercatorScale * EARTH_RADIUS_METERS * log(tan((M_PI * (90 + latitude)) / 360.0));
    float up = altitude;

    T(0) = -left;
    T(1) = up;
    T(2) = fwd;

    return std::move(T);
}