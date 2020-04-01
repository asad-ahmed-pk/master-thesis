//
// MapDatabase.cpp
// Represents the 3D map database and exposes operations for querying the DB
//

#include <unordered_set>

#include "system/MapDataBase.hpp"

namespace System
{
    // Constructor
    MapDataBase::MapDataBase()
    {

    }

    // Add block
    size_t MapDataBase::InsertBlock(std::shared_ptr<MapBlock> block)
    {
        // add to block index map and set ID
        size_t id = m_NextID++;

        m_Blocks[id] = block;
        m_Blocks[id]->SetID(id);
        
        // append to point cloud
        *m_CurrentPointCloud += *block->GetPoints();

        // add bounds to r-tree
        /*
        pcl::PointXYZ min; pcl::PointXYZ max;
        block->GetBoundingBox3D(min, max);

        RTree::Box3D boundingBox(RTree::Point3D(min.x, min.y, min.z), RTree::Point3D(max.x, max.y, max.z));
        m_RTree.insert(std::make_pair(boundingBox, id));
        */

        return id;
    }

    // Block merge
    void MapDataBase::MergeBlocks(const std::vector<std::shared_ptr<MapBlock>>& blocks, const pcl::PointCloud<pcl::PointXYZRGB>& points)
    {
        m_UpdateMutex.lock();
        
        // create new block
        std::vector<std::shared_ptr<TrackingFrame>> keyFrames;
        std::unordered_set<size_t> keyframeIDs;
        
        // add unique keyframes (some keyframes will overlap between blocks)
        for (auto block : blocks)
        {
            // delete the block from the database
            m_Blocks.erase(block->GetID());
            
            // get keyframes from this block
            for (auto keyFrame : block->GetKeyFrames()) {
                if (keyframeIDs.find(keyFrame->GetID()) == keyframeIDs.end()) {
                    keyFrames.push_back(keyFrame);
                }
            }
        }
        
        // create new merged block
        std::shared_ptr<MapBlock> mergedBlock = std::make_shared<MapBlock>(keyFrames, points);
        InsertBlock(mergedBlock);
        
        m_UpdateMutex.unlock();
    }

    // Is Empty
    bool MapDataBase::IsEmpty() const {
        return m_Blocks.empty();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MapDataBase::GetPointCloud() const {
        return m_CurrentPointCloud;
    }

    // Query: overlapping blocks
    void MapDataBase::SelectOverlappingBlocks(const MapBlock& queryBlock, std::vector<std::shared_ptr<MapBlock>>& results)
    {
        pcl::PointXYZ min; pcl::PointXYZ max;
        queryBlock.GetBoundingBox3D(min, max);

        // query R tree
        RTree::Box3D query(RTree::Point3D(min.x, min.y, min.z), RTree::Point3D(max.x, max.y, max.z));
        std::vector<RTree::Value> valueResults;
        m_RTree.query(boost::geometry::index::overlaps(query), std::back_inserter(valueResults));

        // extract pointers to blocks in database
        std::transform(valueResults.begin(), valueResults.end(), std::back_inserter(results), [&](const RTree::Value& v) -> std::shared_ptr<MapBlock> {
            return m_Blocks[v.second];
        });
    }

    // Get full point cloud
    void MapDataBase::GetFullPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const
    {
        for (const auto& pair : m_Blocks) {
            *cloud += *pair.second->GetPoints();
        }
    }
}
