//
// MapDatabase.hpp
// Represents the 3D map database and exposes operations for querying the DB
//

#ifndef MASTER_THESIS_MAPDATABASE_HPP
#define MASTER_THESIS_MAPDATABASE_HPP

#include <unordered_map>
#include <memory>
#include <mutex>
#include <tuple>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "visualisation/PointCloudListener.hpp"
#include "system/MapBlock.hpp"

namespace System
{
    class MapDataBase
    {
    public:
        /// Create a default instance of the database
        MapDataBase();

        ~MapDataBase() = default;

        /// Add a shared reference to the map block to store in the database
        /// \param block The map block
        /// \return The ID of the newly inserted block
        size_t InsertBlock(std::shared_ptr<MapBlock> block);
        
        /// Merge the blocks with the common 3D points
        /// \param blocks The a vector of blocks being merged
        /// \param points The new 3D points to set for this merged block
        void MergeBlocks(const std::vector<std::shared_ptr<MapBlock>>& blocks, const pcl::PointCloud<pcl::PointXYZRGB>& points);
        
        /// Delete the given block with the given ID
        /// \param id The id of the block to delete
        void DeleteBlock(size_t id);

        /// Get a full scene point cloud from all the current blocks in the database
        /// \param cloud Will be populated with all the points from all current blocks
        void GetFullPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;

        /// Check if database is empty
        /// \return True if the database is empty
        bool IsEmpty() const;
        
        /// Get access to the current point cloud being maintained and updated by the database
        /// \return A const shared pointer to the underlying point cloud being actively built and refined
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetPointCloud() const;
        
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetPointCloud(size_t id);
        
        void RegisterAsListener(Visualisation::PointCloudListener* listener);
        
        bool SafeToRead() const;

    private:
        std::unordered_map<size_t, std::shared_ptr<MapBlock>> m_Blocks;
        std::map<size_t, std::tuple<size_t, size_t>> m_PointCloudIndices;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_CurrentPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        std::mutex m_UpdateMutex;
        std::atomic<bool> m_IsUpdatingCurrentPointCloud { false };
        Visualisation::PointCloudListener* m_Listener;         // no ownership
        size_t m_NextID { 0 };
    };
}

#endif //MASTER_THESIS_MAPDATABASE_HPP
