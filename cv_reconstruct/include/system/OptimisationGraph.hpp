//
// OptimisationGraph.hpp
// Wrapper for G2o graph
//

#ifndef OPTIMISATION_GRAPH_HPP
#define OPTIMISATION_GRAPH_HPP

#include <unordered_map>
#include <vector>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/sparse_optimizer.h>

namespace pcl {
    struct PointXYZRGB;
}

namespace System
{
    class VertexSBAPointXYZRGB : public g2o::VertexSBAPointXYZ
    {
    public:
        void SetColor(int r, int g, int b) {
            this->r = r; this->g = g; this->b = b;
        }
        
        Eigen::Vector3i GetColor() {
            return Eigen::Vector3i(r, g, b);
        }
        
    private:
       int r; int g; int b;
   };

    class OptimisationGraph
    {
    public:
        /// Create default instance of optimisation graph with camera intrinsics
        /// \param fx The horizontal focal length
        /// \param fx The vertical focal length
        /// \param cx The horizontal component of the principle point of the camera
        /// \param cy The vertical component of the principle point of the camera
        OptimisationGraph(float fx, float fy, float cx, float cy);
        
        ~OptimisationGraph() = default;
        
        /// Add a default camera pose vertex with the given internal ID
        /// \param isFixed Set to true if this vertex will be fixed
        /// \return The ID of the newly created vertex
        int AddDefaultCameraPoseVertex(bool isFixed);
        
        /// Remove the camera pose vertex with the given ID
        /// \param id The ID to set for the vertex
        void RemoveCameraPoseVertex(int id);
        
        /// Check if camera pose already exists
        /// \param camera The ID of the camera
        bool CameraExists(int camera);
        
        /// Add edges connecting 2 cameras looking at common 3D points
        /// \param cameras The IDs of the pose vertices of the cameras looking at the common 3D points
        /// \param pointsXYZ A vector of observed 3D points for the intial estimate
        /// \param pointsUV A list of lists containing the projections of the 3D points given in pointsXYZ to the camera image of each camera. 
        /// \param isTempEdgeSet Set to true if the edges added for these nodes should be marked as temp
        void AddCamerasLookingAtPoints(const std::vector<int>& cameras, const std::vector<pcl::PointXYZRGB>& pointsXYZ, const std::vector<std::vector<cv::KeyPoint>>& pointsUV, bool isTempEdgeSet);
        
        /// Remove all edges marked as temp
        void RemoveTempEdges();
        
        /// Get the estimated pose of the given camera
        /// \param camera The ID of the camera pose vertex
        /// \return The estimated isomtery for the pose
        Eigen::Isometry3d GetCameraPose(int camera);
        
        /// Get the 3D points observed by the camera with the given ID
        /// \param camera The ID of the camera
        /// \param points Will be populated with points the camera is observing
        void GetPointsObservedByCamera(int camera, std::vector<pcl::PointXYZRGB>& points);
        
        /// Perform optimisation
        /// \param iterations The number of iterations to perform. Default is 10.
        void Optimise(int iterations = 10);
        
        /// Set a camera pose to be fixed
        /// \param camera The ID of the camera pose
        /// \param isFixed Set to true to fix this camera pose
        void SetCameraPoseFixed(int camera, bool isFixed);
        
    private:
        void SetupOptimiser(float fx, float fy, float cx, float cy);
        g2o::EdgeProjectXYZ2UV* AddEdge(g2o::VertexSE3Expmap* poseVertex, g2o::VertexSBAPointXYZ* point3D, float x, float y);
        
    private:
        int m_NextValidVertexID { 0 };
        int m_FixedVertexCount { 0 };
        
    private:
        std::unordered_map<int, std::vector<VertexSBAPointXYZRGB*>> m_CameraToPointsMap;
        std::vector<g2o::EdgeProjectXYZ2UV*> m_TempEdges;
        
    private:
        g2o::SparseOptimizer m_Optimiser;
    };
}

#endif
