//
// SLAMGraph.cpp
// Encapsulates the optimisation graph for map 3D points and camera poses
//

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "slam/Frame.hpp"
#include "slam/SLAMGraph.hpp"

#define NUM_ITERATIONS 3

// Typedefs
namespace SLAM
{
    typedef g2o::BlockSolver_6_3 BlockSolver;
    typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
    typedef g2o::OptimizationAlgorithmLevenberg OptimizationAlgorithm;
}

namespace SLAM
{
    // Constructor
    SLAMGraph::SLAMGraph(const Eigen::Matrix3f& K, const Eigen::VectorXf& d)
    {
        // save K and d as CV matrices
        cv::eigen2cv(K, m_CameraK);
        cv::eigen2cv(d, m_CameraDistCoeffs);

        SetupGraph();
    }

    // Destructor
    SLAMGraph::~SLAMGraph()
    {

    }

    // Graph setup
    void SLAMGraph::SetupGraph()
    {
        m_Optimiser = std::make_unique<g2o::SparseOptimizer>();
        m_Optimiser->setVerbose(false);

        // setup block solver and optimisation algorithm
        LinearSolver* linearSolver { new LinearSolver() };
        BlockSolver* solver { new BlockSolver(linearSolver) };
        g2o::OptimizationAlgorithm* algorithm { new OptimizationAlgorithm(solver) };

        m_Optimiser->setAlgorithm(algorithm);
    }

    // Add a new keyframe
    bool SLAMGraph::AddKeyFrame(const Frame& frame, VertexIndices& vertexIndices)
    {
        // add the camera vertex
        g2o::VertexSE3Expmap* cameraVertex = GetCameraVertex(frame.CameraPose, m_NextVertexIndex, (m_NextVertexIndex == 0));
        if (!m_Optimiser->addVertex(cameraVertex)) {
            return false;
        }
        vertexIndices.CameraPoseVertex = m_NextVertexIndex;
        m_NextVertexIndex++;

        vertexIndices.MapPointVertexStart = m_NextEdgeIndex;

        // add the 3D map points from the point cloud as map vertices
        for (const auto& p : frame.PointCloud->points)
        {
            g2o::VertexSBAPointXYZ* pointVertex = Get3DMapPointVertex(p.x, p.y, p.z, m_NextVertexIndex);
            if (!m_Optimiser->addVertex(pointVertex)) {
                return false;
            }
            m_NextVertexIndex++;

            // add edge connecting this point to cam vertex
            g2o::EdgeProjectXYZ2UV* edge = GetCamToPointEdge(frame.CameraPose, pointVertex, cameraVertex, m_NextEdgeIndex);
            if (m_Optimiser->addEdge(edge)) {
                return false;
            }
            m_NextEdgeIndex++;
        }

        vertexIndices.MapPointVertexEnd = (m_NextEdgeIndex - 1);

        return true;
    }

    // Get 3D point vertex
    inline g2o::VertexSBAPointXYZ* SLAMGraph::Get3DMapPointVertex(float x, float y, float z, int index) const
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ;

        point->setId(index);
        point->setMarginalized(true);
        point->setEstimate(Eigen::Vector3d(x, y, z));

        return point;
    }

    // Get pose vertex from eigen 4x4 pose matrix
    g2o::VertexSE3Expmap* SLAMGraph::GetCameraVertex(const Eigen::Matrix4d pose, int index, bool isFixed) const
    {
        Eigen::Quaterniond q;
        q.setIdentity();

        Eigen::Vector3d t = pose.block(0, 3, 3, 1).cast<double>();
        g2o::SE3Quat quat(q, t);

        g2o::VertexSE3Expmap* poseVertex = new g2o::VertexSE3Expmap;

        poseVertex->setFixed(isFixed);
        poseVertex->setId(index);
        poseVertex->setEstimate(quat);

        return poseVertex;
    }

    // Create and return an edge between the camera and 3D point using the re-projection error as the measurement
    g2o::EdgeProjectXYZ2UV* SLAMGraph::GetCamToPointEdge(const Eigen::Matrix4d& pose, g2o::VertexSBAPointXYZ* mapPointVertex, g2o::VertexSE3Expmap* camVertex, int index) const
    {
        // calculate measurement as the 2D projection of the 3D point
        Eigen::Matrix3d R = pose.block(0, 0, 3, 3).cast<double>();
        cv::Mat rot(3, 3, CV_64F);
        cv::eigen2cv(R, rot);
        std::vector<double> t { pose(0, 3), pose(1, 3), pose(2, 3) };

        cv::Mat point3D = cv::Mat::zeros(1, 3, CV_64F);
        point3D.at<double>(1, 0) = mapPointVertex->estimate().x();
        point3D.at<double>(1, 1) = mapPointVertex->estimate().y();
        point3D.at<double>(1, 2) = mapPointVertex->estimate().z();

        cv::Mat projectedPoint;
        cv::projectPoints(point3D, rot, t, m_CameraK, m_CameraDistCoeffs, projectedPoint);

        Eigen::Vector2d z;
        z(0) = projectedPoint.at<double>(0, 0);
        z(1) = projectedPoint.at<double>(0, 1);

        g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV;

        e->setId(index);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mapPointVertex));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(camVertex));
        e->setMeasurement(z);
        e->information() = Eigen::Matrix2d::Identity();

        return e;
    }

    // Run optimisation
    void SLAMGraph::Optimise()
    {
        m_Optimiser->initializeOptimization();
        m_Optimiser->optimize(NUM_ITERATIONS);
    }
}
