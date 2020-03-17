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
    MapBlock::MapBlock(unsigned long id, const pcl::PointCloud<pcl::PointXYZRGB>& points) : m_ID(id)
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

    // Get bounding box
    void MapBlock::GetBoundingBox3D(pcl::PointXYZ& minPoint, pcl::PointXYZ& maxPoint)
    {
        minPoint.x = m_MinPoint.x;
        minPoint.y = m_MinPoint.y;
        minPoint.z = m_MinPoint.z;

        maxPoint.x = m_MaxPoint.x;
        maxPoint.y = m_MaxPoint.y;
        maxPoint.z = m_MaxPoint.z;
    }
}
