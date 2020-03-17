//
// MapBlock.hpp
// Represents a block for the map (a 'piece' of the map')
//

#ifndef MASTER_THESIS_MAPBLOCK_HPP
#define MASTER_THESIS_MAPBLOCK_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace System
{
    class MapBlock
    {
    public:
        /// Construct a block with the given ID, and set of 3D points
        /// \param id The ID of the block
        /// \param points The localized point cloud for this block
        MapBlock(unsigned long id, const pcl::PointCloud<pcl::PointXYZRGB>& points);

        /// Construct from a copy of another map block
        /// \param otherBlock The other map block
        MapBlock(const MapBlock& otherBlock);

        /// Assignment operator
        /// \param other Other block
        /// \return Reference to this newly assigned block
        MapBlock& operator=(const MapBlock& other);

        ~MapBlock() = default;

        /// Get the 3D bounding box for this map block
        /// \param minPoint The minimum point
        /// \param maxPoint The maximum point
        void GetBoundingBox3D(pcl::PointXYZ& minPoint, pcl::PointXYZ& maxPoint);

    private:
        void CalculateBoundingBox();

    private:
        unsigned long m_ID;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_PointCloud;
        pcl::PointXYZRGB m_MinPoint;
        pcl::PointXYZRGB m_MaxPoint;
    };
}

#endif //MASTER_THESIS_MAPBLOCK_HPP
