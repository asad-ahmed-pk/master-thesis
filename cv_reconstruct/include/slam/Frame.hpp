//
// Frame.hpp
// Represents a frame for the SLAM system: a camera pose and its 3D RGB points
//

#ifndef MASTER_THESIS_SLAM_DATA_STRUCTURES_HPP
#define MASTER_THESIS_SLAM_DATA_STRUCTURES_HPP

#include <eigen3/Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace SLAM
{
    struct Frame
    {
        /// Create a SLAM frame consisting of the camera pose, and the generated 3D points viewed from that camera
        /// \param cameraPose a 6DOF pose in homogenous coordinates
        /// \param pointCloud The generated point cloud with RGB information
        inline Frame(const Eigen::Matrix4d& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud) {
            CameraPose = cameraPose;
            PointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr { new pcl::PointCloud<pcl::PointXYZRGB>() };
            pcl::copyPointCloud(pointCloud, *PointCloud);
        }

        ~Frame()
        {
            if (PointCloud != nullptr) {
                PointCloud->clear();
            }
        }

        Eigen::Matrix4d CameraPose;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud { nullptr };
    };
}

#endif //MASTER_THESIS_SLAM_DATA_STRUCTURES_HPP
