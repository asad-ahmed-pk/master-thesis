//
// SLAMGraph.hpp
// Encapsulates the optimisation graph for map 3D points and camera poses
//

#ifndef MASTER_THESIS_SLAMGRAPH_HPP
#define MASTER_THESIS_SLAMGRAPH_HPP

#include <memory>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

class Frame;
class Map;

namespace SLAM
{
    /// Contains the indices for the newly added camera and map point vertices
    struct VertexIndices {
        int CameraPoseVertex;
        int MapPointVertexStart;
        int MapPointVertexEnd;
    };

    class SLAMGraph
    {
    public:
        /// Construct an instance of the SLAM graph with the given camera parameters
        /// \param K The camera intrinsics
        /// \param d The distortion coefficients
        SLAMGraph(const Eigen::Matrix3f& K, const Eigen::VectorXf& d);

        ~SLAMGraph();

        /// Add the given frame to the graph for optimisation
        /// \param frame The SLAM frame to add to the graph
        /// \param vertexIndices Will be set with the indices of the vertices that were added
        /// \return Returns true on success. False otherwise.
        bool AddKeyFrame(const Frame& frame, VertexIndices& vertexIndices);

        /// Optimise the entire graph
        void Optimise();

        /// Get a pointer to the current map
        /// \return A shared pointer to the current of 3D points being built by the graph
        std::shared_ptr<Map> GetCurrentMap();

    private:
        void SetupGraph();
        g2o::EdgeProjectXYZ2UV* GetCamToPointEdge(const Eigen::Matrix4d& pose, g2o::VertexSBAPointXYZ* mapPointVertex, g2o::VertexSE3Expmap* camVertex, int index) const;
        g2o::VertexSBAPointXYZ* Get3DMapPointVertex(float x, float y, float z, int index) const;
        g2o::VertexSE3Expmap* GetCameraVertex(const Eigen::Matrix4d pose, int index, bool isFixed) const;

    private:
        cv::Mat m_CameraK;
        std::vector<float> m_CameraDistCoeffs;
        int m_NextVertexIndex { 0 };
        int m_NextEdgeIndex { 0 };
        std::unique_ptr<g2o::SparseOptimizer> m_Optimiser;
    };
}

#endif //MASTER_THESIS_SLAMGRAPH_HPP
