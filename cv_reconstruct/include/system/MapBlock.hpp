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

        /// Get the 3D bounding box for this map block
        /// \param minPoint The minimum point
        /// \param maxPoint The maximum point
        void GetBoundingBox3D(pcl::PointXYZ& minPoint, pcl::PointXYZ& maxPoint) const;

        /// Get the center point of the 3D bounding box
        /// \return The center point
        pcl::PointXYZ GetCenterPoint() const;

        /// Get a const pointer to the points stored in this block
        /// \return The const pointer to the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetPoints() const;

        /// Check if this block intersects another
        /// \param other The other map block
        /// \return Returns true if intersects
        bool Intersects(const MapBlock& other) const;

        /// Calculate ratio of overlap with the other block
        /// \param other The other block
        /// \return The ratio of overlap from 0.0 to 1.0
        float OverlapRatio(const MapBlock& other) const;

        /// Get the approximate volume taken by the block
        /// \return The volume of the block
        float GetVolume() const;

        friend std::ostream& operator<<(std::ostream& os, const MapBlock& block);

    private:
        void CalculateBoundingBox();

    private:
        size_t m_ID { 0 };
        std::vector<std::shared_ptr<TrackingFrame>> m_KeyFrames;
        
    private:
        pcl::PointXYZRGB m_MinPoint;
        pcl::PointXYZRGB m_MaxPoint;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_PointCloud;
    };
}

#endif //MASTER_THESIS_MAPBLOCK_HPP
