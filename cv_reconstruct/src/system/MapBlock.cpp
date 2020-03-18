//
// MapBlock.cpp
// Represents a block for the map (a 'piece' of the map')
//

#include <pcl/common/io.h>
#include <pcl/common/common.h>

#include "system/MapBlock.hpp"

namespace System
{
    // Constructor
    MapBlock::MapBlock(const pcl::PointCloud<pcl::PointXYZRGB>& points) : m_ID(-1)
    {
        // copy the cloud
        m_PointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr { new pcl::PointCloud<pcl::PointXYZRGB>() };
        pcl::copyPointCloud(points, *m_PointCloud);

        // determine the bounding box
        CalculateBoundingBox();
    }

    // Copy constructor
    MapBlock::MapBlock(const System::MapBlock& otherBlock) {
        *this = otherBlock;
    }

    // = operator
    MapBlock& MapBlock::operator=(const MapBlock& other)
    {
        if (this != &other)
        {
            m_PointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr { new pcl::PointCloud<pcl::PointXYZRGB>() };
            pcl::copyPointCloud(*other.m_PointCloud, *m_PointCloud);

            m_ID = other.m_ID;

            m_MinPoint = other.m_MinPoint;
            m_MaxPoint = other.m_MaxPoint;
        }

        return *this;
    }

    // Calculate bounding box
    void MapBlock::CalculateBoundingBox() {
        pcl::getMinMax3D(*m_PointCloud, m_MinPoint, m_MaxPoint);
    }

    // Set ID
    void MapBlock::SetID(size_t id) {
        m_ID = id;
    }

    // Get ID
    size_t MapBlock::GetID() const {
        return m_ID;
    }

    // Get center point
    pcl::PointXYZ MapBlock::GetCenterPoint() const
    {
        pcl::PointXYZ mid;

        mid.x = (m_MinPoint.x + m_MaxPoint.x) / 2.0f;
        mid.y = (m_MinPoint.y + m_MaxPoint.y) / 2.0f;
        mid.z = (m_MinPoint.z + m_MaxPoint.z) / 2.0f;

        return mid;
    }

    // Get bounding box
    void MapBlock::GetBoundingBox3D(pcl::PointXYZ& minPoint, pcl::PointXYZ& maxPoint) const
    {
        minPoint.x = m_MinPoint.x;
        minPoint.y = m_MinPoint.y;
        minPoint.z = m_MinPoint.z;

        maxPoint.x = m_MaxPoint.x;
        maxPoint.y = m_MaxPoint.y;
        maxPoint.z = m_MaxPoint.z;
    }

    // Get points
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MapBlock::GetPoints() const {
        return m_PointCloud;
    }

    // Intersection check
    bool MapBlock::Intersects(const MapBlock& other) const
    {
        return (m_MinPoint.x <= other.m_MaxPoint.x && m_MaxPoint.x >= other.m_MinPoint.x) &&
        (m_MinPoint.y <= other.m_MaxPoint.y && m_MaxPoint.y >= other.m_MinPoint.y) &&
        (m_MinPoint.z <= other.m_MaxPoint.z && m_MaxPoint.z >= other.m_MinPoint.z);
    }

    // Overlap ratio
    float MapBlock::OverlapRatio(const System::MapBlock& other) const
    {
        if (!Intersects(other)) {
            return 0.0;
        }

        float v1 = (m_MaxPoint.x - m_MinPoint.x) * (m_MaxPoint.y - m_MinPoint.y) * (m_MaxPoint.z - m_MinPoint.z);
        float v2 = (other.m_MaxPoint.x - other.m_MinPoint.x) * (other.m_MaxPoint.y - other.m_MinPoint.y) * (other.m_MaxPoint.z - other.m_MinPoint.z);

        return (fabs(v2 - v1) / (v1 + v2));
    }
}
