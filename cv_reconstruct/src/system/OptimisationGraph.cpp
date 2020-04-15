//
// OptimisationGraph.cpp
// Wrapper for G2o graph
//

#include <iostream>

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>

#include "system/OptimisationGraph.hpp"

namespace System
{
    /// Constructor
    OptimisationGraph::OptimisationGraph(float fx, float fy, float cx, float cy) {
        SetupOptimiser(fx, fy, cx, cy);
    }

    // Optimiser setup
    void OptimisationGraph::SetupOptimiser(float fx, float fy, float cx, float cy)
    {
        // algorithm setup
        g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3* blockSolver = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
        m_Optimiser.setAlgorithm(algorithm);
        m_Optimiser.setVerbose(false);
        
        // camera parameters
        g2o::CameraParameters* cameraParams = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
        cameraParams->setId(0);
        m_Optimiser.addParameter(cameraParams);
    }

    // Add empty vertex
    int OptimisationGraph::AddDefaultCameraPoseVertex(bool isFixed)
    {
        int id = m_NextValidVertexID++;
        
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        
        v->setId(id);
        v->setFixed(isFixed);
        v->setEstimate(g2o::SE3Quat());
        
        m_Optimiser.addVertex(v);
        
        return id;
    }

    // Camera vertex check
    bool OptimisationGraph::CameraExists(int camera) {
        return (m_Optimiser.vertex(camera) != nullptr);
    }

    // Remove pose vertex
    void OptimisationGraph::RemoveCameraPoseVertex(int id)
    {
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(m_Optimiser.vertex(id));
        
        if (v != nullptr)
        {
            if (v->fixed()) {
                m_FixedVertexCount--;
            }
            
            m_Optimiser.removeVertex(v);
            
            auto iter = m_CameraToPointsMap.find(id);
            if (iter != m_CameraToPointsMap.end()) {
                m_CameraToPointsMap.erase(iter);
            }
        }
    }

    // Remove temp edges
    void OptimisationGraph::RemoveTempEdges() {
        for (g2o::EdgeProjectXYZ2UV* edge : m_TempEdges) {
            m_Optimiser.removeEdge(edge);
        }
    }

    // Add camera pair with edges looking at 3D points
    void OptimisationGraph::AddCamerasLookingAtPoints(const std::vector<int>& cameras, const std::vector<pcl::PointXYZRGB>& pointsXYZ, const std::vector<std::vector<cv::KeyPoint>>& pointsUV, bool isTempEdgeSet)
    {
        // create 3D point vertices that cameras are looking at
        int pointVertexID = 0;
        std::vector<VertexSBAPointXYZRGB*> pointVertices;
        for (size_t i = 0; i < pointsXYZ.size(); i++)
        {
            pointVertexID = m_NextValidVertexID++;
            
            //g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
            VertexSBAPointXYZRGB* v = new VertexSBAPointXYZRGB();
            v->setId(pointVertexID);
            v->setMarginalized(true);
            v->setEstimate(Eigen::Vector3d(pointsXYZ[i].x, pointsXYZ[i].y, pointsXYZ[i].z));
            
            // colour information
            Eigen::Vector3i color = pointsXYZ[i].getRGBVector3i();
            v->SetColor(color(0), color(1), color(2));
            
            m_Optimiser.addVertex(v);
            pointVertices.push_back(v);
        }
        
        // get pose vertices for the cameras
        std::vector<g2o::VertexSE3Expmap*> poses;
        for (int id : cameras)
        {
            g2o::VertexSE3Expmap* pose = dynamic_cast<g2o::VertexSE3Expmap*>(m_Optimiser.vertex(id));
            if (pose != nullptr) {
                poses.push_back(pose);
                m_CameraToPointsMap[id] = pointVertices;
            }
            else {
                std::cerr << "\nWarning: No pose vertex with ID " << id << " exists!" << std::endl;
            }
        }
        
        // assuming these cameras are all looking at the same 3D points...
        // create edges connecting poses to these 3D points
        for (size_t i = 0; i < poses.size(); i++)
        {
            for (size_t j = 0; j < pointsXYZ.size(); j++)
            {
                g2o::EdgeProjectXYZ2UV* edge = AddEdge(poses[i], pointVertices[j], pointsUV[i][j].pt.x, pointsUV[i][j].pt.y);
                if (isTempEdgeSet) {
                    m_TempEdges.push_back(edge);
                }
            }
        }
    }

    // Set fixed
    void OptimisationGraph::SetCameraPoseFixed(int camera, bool isFixed)
    {
        g2o::VertexSE3Expmap* cameraVertex = dynamic_cast<g2o::VertexSE3Expmap*>(m_Optimiser.vertex(camera));
        if (cameraVertex != nullptr) {
            cameraVertex->setFixed(isFixed);
        }
    }

    // Add edge for camera pose to 3D point observations
    g2o::EdgeProjectXYZ2UV* OptimisationGraph::AddEdge(g2o::VertexSE3Expmap* poseVertex, g2o::VertexSBAPointXYZ* point3D, float x, float y)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        
        edge->setVertex(0, point3D);
        edge->setVertex(1, poseVertex);
        edge->setMeasurement(Eigen::Vector2d(x, y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        
        m_Optimiser.addEdge(edge);
        
        return edge;
    }

    // Get pose of camera
    Eigen::Isometry3d OptimisationGraph::GetCameraPose(int camera)
    {
        // get the pose vertex from the graph
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(m_Optimiser.vertex(camera));
        if (v != nullptr) {
            Eigen::Isometry3d pose = v->estimate();
            return pose;
        }
        else {
            std::cerr << "Error: Invalid camera ID: " << camera << "!" << std::endl;
            return Eigen::Isometry3d::Identity();
        }
    }

    // Get points camera sees
    void OptimisationGraph::GetPointsObservedByCamera(int camera, std::vector<pcl::PointXYZRGB>& points)
    {
        if (m_CameraToPointsMap.find(camera) != m_CameraToPointsMap.end())
        {
            Eigen::Vector3d point;
            Eigen::Vector3i color;
            pcl::PointXYZRGB colorPoint;
            
            for (VertexSBAPointXYZRGB* pointVertex : m_CameraToPointsMap[camera])
            {
                color = pointVertex->GetColor();
                point = pointVertex->estimate();
                
                colorPoint = pcl::PointXYZRGB(color(0), color(1), color(2));
                colorPoint.x = point(0);
                colorPoint.y = point(1);
                colorPoint.z = point(2);
                
                points.push_back(colorPoint);
            }
        }
    }

    // Optimisation
    void OptimisationGraph::Optimise(int iterations) {
        m_Optimiser.initializeOptimization();
        m_Optimiser.optimize(iterations);
    }
}
