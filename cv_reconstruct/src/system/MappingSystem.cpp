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

    // Add point cloud
    bool MappingSystem::AddPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& points)
    {
        // create map block from points
        std::shared_ptr<MapBlock> block = std::make_shared<MapBlock>(points);

        // insert first block
        if (m_MapDataBase.IsEmpty())
        {
            size_t id = m_MapDataBase.InsertBlock(block);
            block->SetID(id);
            std::cout << "\nFirst Block inserted into database" << std::endl;
            std::cout << *block << std::endl;
            return true;
        }
        else {
            // for now: just insert it anyway
            size_t id = m_MapDataBase.InsertBlock(block);
            block->SetID(id);
            return true;
        }

        /*
        // get overlapping blocks from database system and select last inserted one (largest ID)
        std::vector<std::shared_ptr<MapBlock>> results;
        m_MapDataBase.SelectOverlappingBlocks(*block, results);

        std::sort(results.begin(), results.end(), [](const std::shared_ptr<MapBlock>& p1, const std::shared_ptr<MapBlock>& p2) -> bool {
            return (p1->GetID() > p2->GetID());
        });

        if (results.empty()) {
            std::cout << "\nNo overlapping blocks found for new block" << std::endl;
            std::cout << *block << std::endl;
            return false;
        }

        // get last overlapping block
        std::shared_ptr<MapBlock> lastOverlappingBlock = results[0];
        float overlapRatio = block->OverlapRatio(*lastOverlappingBlock);

        if (overlapRatio >= MIN_OVERLAP_RATIO_NEDDED_FOR_INSERT && overlapRatio <= MAX_OVERLAP_RATIO_NEDDED_FOR_INSERT) {
            size_t id = m_MapDataBase.InsertBlock(block);
            std::cout << "\nBlock #" << id << " inserted due to overlap (" << overlapRatio << ") with block #" << lastOverlappingBlock->GetID() << std::endl;
            return true;
        }
        else {
            std::cout << "\nNew block failed to meet insertion criteria with overlap of " << overlapRatio << " with block #" << lastOverlappingBlock->GetID() << std::endl;
        }

        return false;
        */
    }

    // Get map
    void MappingSystem::GetMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
        m_MapDataBase.GetFullPointCloud(cloud);
    }
}
