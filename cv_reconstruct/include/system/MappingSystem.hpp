//
// MappingSystem.hpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#ifndef MASTER_THESIS_MAPPINGSYSTEM_HPP
#define MASTER_THESIS_MAPPINGSYSTEM_HPP

#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "system/MapDataBase.hpp"

namespace System
{
    class TrackingFrame;

    class MappingSystem
    {
    public:
        /// Create default instance of mapping system.
        MappingSystem();

        /// Start the optimisation thread
        void StartOptimisationThread();

        ~MappingSystem() = default;

        /// Add the given points to the mapping system to process
        /// \param points The potentially new point cloud to add to the underlying map
        /// \param keyFrames The keyframes that are observing these points
        void AddPointsForKeyFrames(const pcl::PointCloud<pcl::PointXYZRGB>& points, const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames);

        /// Get the current built map
        /// \param cloud Will be filled with the points of the current map
        void GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;

    private:
        MapDataBase m_MapDataBase;
    };
}

#endif //MASTER_THESIS_MAPPINGSYSTEM_HPP
