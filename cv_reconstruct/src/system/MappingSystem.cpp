//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include <memory>

#include "system/MappingSystem.hpp"
#include "system/MapBlock.hpp"

#define MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT 0.3f
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

    // Add point cloud
    void MappingSystem::AddPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& points)
    {
        // create map block from points
        std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(points);

        // get overlapping blocks from database system
        std::vector<std::shared_ptr<MapBlock>> results;
        m_MapDataBase.SelectOverlappingBlocks(*block, results);

        // check % overlap and insert if meets criteria for an insert
        // criteria: overlaps with at least 1 other block in the map
        float overlapRatio = 0.0f;
        for (const auto& overlappingBlock : results)
        {
            overlapRatio = block->OverlapRatio(*overlappingBlock);
            if (overlapRatio >= MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT && overlapRatio <= MAX_OVERLAP_RATIO_NEDDED_FOR_INSERT) {
                m_MapDataBase.InsertBlock(block);
                break;
            }
        }
    }
}