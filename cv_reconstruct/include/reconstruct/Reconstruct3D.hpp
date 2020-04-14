//
// Reconstruct3D.hpp
// Module responsible for 3D reconstruction of stereo images
//

#ifndef CV_RECONSTRUCT_RECONSTRUCT3D_HPP
#define CV_RECONSTRUCT_RECONSTRUCT3D_HPP

#include "camera/CameraCalib.hpp"
#include "pipeline/StereoFrame.hpp"
#include "config/Config.hpp"
#include "Localizer.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
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
        Reconstruct3D(const Camera::Calib::StereoCalib& stereoSetup, const Config::Config& config);

        /// Generate the disparity map for the given stereo images
        /// \param leftImage The left camera image
        /// \param rightImage The right camera image
        /// \return The disparity map
        cv::Mat GenerateDisparityMap(const cv::Mat& leftImage, const cv::Mat& rightImage) const;

        /// Generate point cloud using triangulation method
        /// \param disparity The disparity image (parallax map)
        /// \param cameraImage The RGB camera image (rectified)
        /// \return Returns the generated point cloud with RGB information
        pcl::PointCloud<pcl::PointXYZRGB> Triangulate3D(const cv::Mat& disparity, const cv::Mat& cameraImage, cv::InputArray& mask = cv::noArray()) const;
        
        /// Triangulate the single image point to 3D space
        /// \param u The x coorindate in the image plane
        /// \param v The y coordinate in the image plane
        /// \param disparity The disparity computed for this image point
        /// \param color The RGB color triplet for this image
        /// \return The triangulated 3D point with color information
        pcl::PointXYZRGB TriangulateUV(float u, float v, float disparity, const cv::Vec3b& color) const;
        
        /// Backproject a point in 2D to 3D
        /// \param x the x coord of the point
        /// \param y the y coord of the point
        /// \return the 3D projected point
        pcl::PointXYZ BackProjectPoint(float x, float y) const;
        
        /// Triangulate the list of 2D image points using the given dispartiy image
        /// \param disparity The disparity image
        /// \param cameraImage The camera image (3 channel 8 bit)
        /// \param points The list of 2D image points
        /// \param triangulatedPoints Output vector that will be populated with 3D points corresponding to each 2D image point in points vector
        void TriangulatePoints(const cv::Mat& disparity, const cv::Mat& cameraImage, const std::vector<cv::KeyPoint>& points, std::vector<pcl::PointXYZRGB>& triangulatedPoints) const;
        
        /// Get camera itrinsics for selected camera number
        /// \param fx The horizontal focal length
        /// \param fy The vertical focal length
        /// \param cx The horizontal component of the principle point
        /// \param cy The vertical component of the principle point
        /// \param camNumber The camera number, default is 0 - left camera
        void GetCameraParameters(float& fx, float& fy, float& cx, float& cy, int camNumber = 0) const;

        /// Apply stereo rectification to the images
        /// \param leftImage The left camera image (rectified)
        /// \param rightImage The right camera image (rectified)
        /// \param rectLeftImage Will be updated with the rectified image for the left camera
        /// \param rectRightImage Will be updated with the rectified image for the right camera
        void RectifyImages(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& rectLeftImage, cv::Mat& rectRightImage) const;

        /// Set the window size for the block matcher for computing disparity
        /// \param size The window size (odd number)
        void SetStereoBMWindowSize(int size);

        /// Set the number of disparity levels for the computed disparity images
        /// \param num Number of disparities (multiple of 16)
        void SetStereoBMNumDisparities(int num);

        /// Set the type of block matcher that will be used to compute the disparity
        /// \param type The type of block matcher to use
        void SetBlockMatcherType(StereoBlockMatcherType type);

    private:
        void ConfigureSteoreoMatcher(const Config::Config& config);

    private:
        Camera::Calib::StereoCalib m_StereoCameraSetup;
        cv::Ptr<cv::StereoMatcher> m_StereoMatcher { nullptr };
        StereoBlockMatcherType m_StereoBlockMatcherType { STEREO_BLOCK_MATCHER };
    };
}

#endif //CV_RECONSTRUCT_RECONSTRUCT3D_HPP
