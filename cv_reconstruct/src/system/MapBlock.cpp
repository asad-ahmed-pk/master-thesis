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
        }

        return *this;
    }

    // Set ID
    void MapBlock::SetID(size_t id) {
        m_ID = id;
    }

    // Get ID
    size_t MapBlock::GetID() const {
        return m_ID;
    }

    // Get keyframes
    std::vector<std::shared_ptr<TrackingFrame>> MapBlock::GetKeyFrames() const {
        return m_KeyFrames;
    }

    // Get points
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MapBlock::GetPoints() const {
        return m_PointCloud;
    }
}
