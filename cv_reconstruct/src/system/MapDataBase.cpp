//
// MapDatabase.cpp
// Represents the 3D map database and exposes operations for querying the DB
//

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

        // add bounds to r-tree
        /*
        pcl::PointXYZ min; pcl::PointXYZ max;
        block->GetBoundingBox3D(min, max);

        RTree::Box3D boundingBox(RTree::Point3D(min.x, min.y, min.z), RTree::Point3D(max.x, max.y, max.z));
        m_RTree.insert(std::make_pair(boundingBox, id));
        */

        return id;
    }

    // Is Empty
    bool MapDataBase::IsEmpty() const {
        return m_Blocks.empty();
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