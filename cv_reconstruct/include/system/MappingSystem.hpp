//
// MappingSystem.hpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#ifndef MASTER_THESIS_MAPPINGSYSTEM_HPP
#define MASTER_THESIS_MAPPINGSYSTEM_HPP

#include <vector>
#include <memory>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/sba/types_six_dof_expmap.h>

#include "pipeline/OpticalFlowEstimator.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "system/TrackingFrame.hpp"
#include "system/OptimisationGraph.hpp"
#include "system/MapBlock.hpp"
#include "system/MapDataBase.hpp"
#include "system/KeyFrameDatabase.hpp"

namespace System
{
    class TrackingFrame;

    class MappingSystem
    {
    public:
        /// Create default instance of mapping system.
        MappingSystem(std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor, std::shared_ptr<KeyFrameDatabase> keyFrameDB);

        /// Start the optimisation thread
        void StartOptimisationThread();

        ~MappingSystem() = default;
        
        void AddKeyFrame(std::shared_ptr<TrackingFrame> keyFrame);
        
        void AddKeyFrames(const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames);

        /// Add the given points to the mapping system to process
        /// \param points The potentially new point cloud to add to the underlying map
        /// \param keyFrames The keyframes that are observing these points
        void AddPointsForKeyFrames(const pcl::PointCloud<pcl::PointXYZRGB>& points, const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames);

        /// Get the current built map
        /// \param cloud Will be filled with the points of the current map
        void GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
        
        /// Get a shared pointer to the map database
        /// \return A shared pointer to the map database
        std::shared_ptr<MapDataBase> GetMapDataBase() const;
        
        void FullBA();
        
    private:
        void LocalOptimisation();

    private:
        std::unordered_map<size_t, int> m_CameraGraphIDs;
        std::vector<std::shared_ptr<MapBlock>> m_UnoptimisedBlocks;
        std::shared_ptr<MapBlock> m_TargetBlock { nullptr };
        Eigen::Matrix4f m_TransformLast = Eigen::Matrix4f::Identity();
        float deltaX = 0; float deltaZ = 0;
        
    private:
        size_t m_FixedKeyFrameIndex { 0 };
        std::unordered_map<size_t, g2o::SE3Quat> m_KeyFramePoses;
        std::vector<std::shared_ptr<TrackingFrame>> m_KeyFramesForOptimisation;
        std::vector<std::shared_ptr<TrackingFrame>> m_KeyFrames;
        
    private:
        std::unique_ptr<OptimisationGraph> m_OptimisationGraph;
        std::unique_ptr<Features::OpticalFlowEstimator> m_OpticalFlowEstimator;
        std::shared_ptr<Reconstruct::Reconstruct3D> m_3DReconstructor;
        std::shared_ptr<MapDataBase> m_MapDataBase;
        std::shared_ptr<KeyFrameDatabase> m_KeyFrameDataBase;
        std::shared_ptr<TrackingFrame> m_LastKeyFrameAddedToMap;
    };
}

#endif //MASTER_THESIS_MAPPINGSYSTEM_HPP
