//
// MapBlock.cpp
// Represents a block for the map (a 'piece' of the map')
//

#include <cmath>

#include <pcl/common/io.h>
#include <pcl/common/common.h>

#include "system/MapBlock.hpp"

namespace System
{
    // Constructor
    MapBlock::MapBlock(const std::vector<std::shared_ptr<TrackingFrame>>& keyFrames, const pcl::PointCloud<pcl::PointXYZRGB>& points) : m_KeyFrames(keyFrames)
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

        mid.x = (m_MaxPoint.x - m_MinPoint.x) / 2.0f;
        mid.y = (m_MaxPoint.y - m_MinPoint.y) / 2.0f;
        mid.z = (m_MaxPoint.z - m_MinPoint.z) / 2.0f;

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

        float va = GetVolume();
        float vb = other.GetVolume();

        float xa1 = m_MinPoint.x; float xa2 = m_MaxPoint.x;
        float ya1 = m_MinPoint.y; float ya2 = m_MaxPoint.y;
        float za1 = m_MinPoint.z; float za2 = m_MaxPoint.z;

        float xb1 = other.m_MinPoint.x; float xb2 = other.m_MaxPoint.x;
        float yb1 = other.m_MinPoint.y; float yb2 = other.m_MaxPoint.y;
        float zb1 = other.m_MinPoint.z; float zb2 = other.m_MaxPoint.z;

        float intersectionVolume = fmax(0.0f, fmin(xa2, xb2) - fmax(xa1, xb1)) * fmax(0.0f, fmin(ya2, yb2) - fmax(ya1, yb1)) * fmax(0.0f, fmin(za2, zb2) - fmax(za1, zb1));
        float unionVolume = va + vb - intersectionVolume;

        return (intersectionVolume / unionVolume);
    }

    // Get volume
    float MapBlock::GetVolume() const {
        return ((m_MaxPoint.x - m_MinPoint.x) * (m_MaxPoint.y - m_MinPoint.y) * (m_MaxPoint.z - m_MinPoint.z));
    }

    // << operator
    std::ostream& operator <<(std::ostream& os, const MapBlock& block)
    {
        os << "\nBlock #" << block.m_ID;
        os << "\nMin: (" << block.m_MinPoint.x << ", " << block.m_MinPoint.y << ", " << block.m_MinPoint.z << ")";
        os << "\nMax: (" << block.m_MaxPoint.x << ", " << block.m_MaxPoint.y << ", " << block.m_MaxPoint.z << ")";
        os << "\nPoints: " << block.m_PointCloud->size();

        return os;
    }
}
