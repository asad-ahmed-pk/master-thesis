//
// Map.hpp
// Maps the 3D points and colour information. Holds data on the 3D map
//

#ifndef MASTER_THESIS_MAP_HPP
#define MASTER_THESIS_MAP_HPP

#include <unordered_map>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <pcl/point_types.h>

namespace SLAM
{
    class Map
    {
    public:
        Map() = default;

        ~Map() = default;

        /// Add a camera pose to the map
        /// \param pose The camera pose in homogenous coordinates
        void AddCameraPose(const Eigen::Matrix4d pose);

        /// Add the given 3D point to the map
        /// \param id The global ID of the point being added
        /// \param point The 3D point with RGB data
        void AddMapPointRGB(int id, const pcl::PointXYZRGB& point);

        /// Get a 'slice' of the 3D map for the last n key frames
        /// \param points Vector to be filled with the 3D points
        /// \param n The number of previous frames to be considered
        void GetMapSlice(std::vector<pcl::PointXYZRGB>& points, int n = 6) const;

    private:
        std::vector<Eigen::Matrix4d> m_CameraPoses;
        std::unordered_map<int, Eigen::Vector3i> m_RGBMap;
    };
}

#endif //MASTER_THESIS_MAP_HPP
