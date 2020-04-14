//
// MapBlock.hpp
// Represents a block for the map (a 'piece' of the map')
//

#ifndef MASTER_THESIS_MAPBLOCK_HPP
#define MASTER_THESIS_MAPBLOCK_HPP

#include <iostream>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "system/TrackingFrame.hpp"

namespace System
{
    class MapBlock
    {
    public:
        /// Construct a block with the given ID, and set of 3D points
        /// \param keyFrames the keyframes that are looking at these points
        /// \param points The localized point cloud for this block
        MapBlock(const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames, const pcl::PointCloud<pcl::PointXYZRGB>& points);

        /// Construct from a copy of another map block
        /// \param otherBlock The other map block
        MapBlock(const MapBlock& otherBlock);

        /// Assignment operator
        /// \param other Other block
        /// \return Reference to this newly assigned block
        MapBlock& operator=(const MapBlock& other);

        ~MapBlock() = default;

        /// Set the ID for this block
        /// \param id The ID for the block
        void SetID(size_t id);

        /// Get the ID of this block
        /// \return The ID
        size_t GetID() const;
        
        /// Get keyframes
        /// \return The keyframes in this block
        std::vector<std::shared_ptr<TrackingFrame>> GetKeyFrames() const;

        /// Get a const pointer to the points stored in this block
        /// \return The const pointer to the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetPoints() const;

    private:
        size_t m_ID { 0 };
        std::vector<std::shared_ptr<TrackingFrame>> m_KeyFrames;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_PointCloud;
    };
}

#endif //MASTER_THESIS_MAPBLOCK_HPP
