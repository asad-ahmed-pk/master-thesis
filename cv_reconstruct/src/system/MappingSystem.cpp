//
// MappingSystem.cpp
// 3D mapping system that handles the development and refinement of the 3D map
//

#include "system/MappingSystem.hpp"

namespace System
{
    // Constructor
    MappingSystem::MappingSystem()
    {

    }

    // Start optimisation thread
    void MappingSystem::StartOptimsationThread()
    {
        // TODO: launch background thread for map refinement
    }

    // Add point cloud
    void MappingSystem::AddPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& points)
    {
        // TODO: add to database if 3D space is empty 


    }
}