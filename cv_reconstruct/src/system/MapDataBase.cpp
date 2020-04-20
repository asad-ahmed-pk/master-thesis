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

    bool MapDataBase::SafeToRead() const {
        return !m_IsUpdatingCurrentPointCloud;
    }

    // Listener
    void MapDataBase::RegisterAsListener(Visualisation::PointCloudListener* listener) {
        m_Listener = listener;
    }

    // Add block
    size_t MapDataBase::InsertBlock(std::shared_ptr<MapBlock> block)
    {
        m_UpdateMutex.lock();
        
        // add to block index map and set ID
        size_t id = m_NextID++;

        m_Blocks[id] = block;
        m_Blocks[id]->SetID(id);
        
        // track the indices of the points for this cloud
        size_t startIndex = m_CurrentPointCloud->size();
        size_t endIndex = startIndex + block->GetPoints()->size();
        m_PointCloudIndices[id] = std::make_tuple(startIndex, endIndex);
        
        // append to point cloud
        *m_CurrentPointCloud += *block->GetPoints();
        
        m_UpdateMutex.unlock();
        
        // notify listener
        if (m_Listener != nullptr) {
            m_Listener->PointCloudWasAdded(block->GetID(), block->GetPoints());
        }

        return id;
    }

    // Delete block
    void MapDataBase::DeleteBlock(size_t id)
    {
        m_UpdateMutex.lock();
        
        auto iter = m_Blocks.find(id);
        if (iter != m_Blocks.end())
        {
            m_Blocks.erase(iter);
        
            // remove indices and points from point cloud
            auto indexIter = m_PointCloudIndices.find(id);
            if (indexIter != m_PointCloudIndices.end())
            {
                size_t startIndex = std::get<0>(m_PointCloudIndices[id]);
                size_t endIndex = std::get<1>(m_PointCloudIndices[id]);
                size_t numPointsDeleted = endIndex - startIndex;
                
                // delete the points from the point cloud
                m_CurrentPointCloud->erase(m_CurrentPointCloud->begin() + startIndex, m_CurrentPointCloud->begin() + endIndex);
                m_PointCloudIndices.erase(indexIter);
                
                // adjust all indices after this blocks ID
                for (auto iter = m_PointCloudIndices.begin(); iter != m_PointCloudIndices.end(); iter++)
                {
                    if (iter->first > id) {
                        std::get<0>(iter->second) = std::get<0>(iter->second) - numPointsDeleted;
                        std::get<1>(iter->second) = std::get<1>(iter->second) - numPointsDeleted;
                    }
                }
            }
        }
        
        m_UpdateMutex.unlock();
        
        // notify listener
        if (m_Listener != nullptr) {
            m_Listener->PointCloudWasDeleted(id);
        }
    }

    // Block merge
    void MapDataBase::MergeBlocks(const std::vector<std::shared_ptr<MapBlock>>& blocks, const pcl::PointCloud<pcl::PointXYZRGB>& points)
    {
        // create new block
        std::vector<std::shared_ptr<TrackingFrame>> keyFrames;
        std::unordered_set<size_t> keyframeIDs;
        
        // add unique keyframes (some keyframes will overlap between blocks)
        for (auto block : blocks)
        {
            // get keyframes from this block
            for (auto keyFrame : block->GetKeyFrames()) {
                if (keyframeIDs.find(keyFrame->GetID()) == keyframeIDs.end()) {
                    keyFrames.push_back(keyFrame);
                }
            }
        }
        
        m_IsUpdatingCurrentPointCloud = true;
        
        // create new merged block
        std::shared_ptr<MapBlock> mergedBlock = std::make_shared<MapBlock>(keyFrames, points);
        InsertBlock(mergedBlock);
        
        // delete the old blocks from the database as these have now been merged into a new one
        for (auto block : blocks) {
            DeleteBlock(block->GetID());
        }
        
        m_IsUpdatingCurrentPointCloud = false;
    }

    // Is Empty
    bool MapDataBase::IsEmpty() const {
        return m_Blocks.empty();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MapDataBase::GetPointCloud(size_t id) {
        return m_Blocks[id]->GetPoints();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MapDataBase::GetPointCloud() const {
        return m_CurrentPointCloud;
    }

    // Get full point cloud
    void MapDataBase::GetFullPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const
    {
        // perform full BA
        for (const auto& pair : m_Blocks) {
            *cloud += *pair.second->GetPoints();
        }
    }
}
