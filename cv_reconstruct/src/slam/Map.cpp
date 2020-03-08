//
// Map.hpp
// Maps the 3D points and colour information. Holds data on the 3D map
//

#include "slam/Map.hpp"

namespace SLAM
{
    // Add camera pose
    void Map::AddCameraPose(const Eigen::Matrix4d pose) {
        m_CameraPoses.push_back(pose);
    }

    // Add 3D map point's RGB data
    void Map::AddMapPointRGB(int id, const pcl::PointXYZRGB& point) {
        Eigen::Vector3i rgb = point.getRGBVector3i();
        m_RGBMap[id] = std::move(rgb);
    }

    // Get map points
    void Map::GetMapSlice(std::vector<pcl::PointXYZRGB>& points, int n) const
    {
        // TODO: get map points for last n poses...
    }
}
