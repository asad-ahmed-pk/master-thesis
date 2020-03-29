//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include <memory>

#include "system/MappingSystem.hpp"
#include "system/MapBlock.hpp"

#define MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.2f
#define MAX_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.6f

namespace System
{
    // Constructor
    MappingSystem::MappingSystem()
    {

    }

    // Start optimisation thread
    void MappingSystem::StartOptimisationThread()
    {
        // TODO: launch background thread for map refinement
    }

    // Add common point cloud seen from keyframes
    void MappingSystem::AddPointsForKeyFrames(const pcl::PointCloud<pcl::PointXYZRGB>& points, const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames)
    {
        // create map block from points and insert into database
        std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(keyFrames, points);
        m_MapDataBase.InsertBlock(block);
        
        std::cout << "\nMapping system added 3D points" << std::endl;
    }

    // Get map
    void MappingSystem::GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
        m_MapDataBase.GetFullPointCloud(cloud);
    }
}
