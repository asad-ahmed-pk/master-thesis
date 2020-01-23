//
// Reconstruct3D.hpp
// Module responsible for 3D reconstruction of stereo images
//

#ifndef CV_RECONSTRUCT_RECONSTRUCT3D_HPP
#define CV_RECONSTRUCT_RECONSTRUCT3D_HPP

#include "camera/CameraCalib.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace Reconstruct
{
    class Reconstruct3D
    {
    public:
        /// Create a 3D reconstructor for the given stereo rig
        /// \param stereoSetup The calibrated, stereo rig setup with stereo rectification already applied
        explicit Reconstruct3D(const Camera::Calib::StereoCalib& stereoSetup);

        /// Generate the disparity map for the given stereo images
        /// \param leftImage The left camera image
        /// \param rightImage The right camera image
        /// \return The disparity map
        cv::Mat GenerateDisparityMap(const cv::Mat& leftImage, const cv::Mat& rightImage) const;

        /// Generate point cloud from disparity map
        /// \param disparity The 8 or 16 bit disparity image
        /// \param cameraImage A RGB camera image used for creating the disparity map
        /// \return The generated point cloud with X,Y,Z
        pcl::PointCloud<pcl::PointXYZRGB> GeneratePointCloud(const cv::Mat& disparity, const cv::Mat& cameraImage) const;

    private:
        Camera::Calib::StereoCalib m_StereoCameraSetup;
        cv::Ptr<cv::StereoSGBM> m_StereoMatcher;
    };
}

#endif //CV_RECONSTRUCT_RECONSTRUCT3D_HPP
