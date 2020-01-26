//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "reconstruct/Reconstruct3D.hpp"

#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>

namespace Reconstruct
{
    const int SGM_MIN_DISPARITY = 0;
    const int SGM_BLOCK_SIZE = 11;
    const int SGM_NUM_DISPARITIES = 32;

    constexpr int SGM_P1 = 8 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;
    constexpr int SGM_P2 = 32 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;

    // Constructor
    Reconstruct3D::Reconstruct3D(const Camera::Calib::StereoCalib& stereoSetup) : m_StereoCameraSetup(stereoSetup)
    {
        // setup stereo matcher
        m_StereoMatcher = cv::StereoSGBM::create(SGM_MIN_DISPARITY, SGM_NUM_DISPARITIES, SGM_BLOCK_SIZE, SGM_P1, SGM_P2);

        m_StereoMatcher->setUniquenessRatio(10);
        m_StereoMatcher->setSpeckleRange(2);
        m_StereoMatcher->setSpeckleWindowSize(128);
        m_StereoMatcher->setDisp12MaxDiff(1);
        m_StereoMatcher->setPreFilterCap(31);
    }

    // Disparity map
    cv::Mat Reconstruct3D::GenerateDisparityMap(const cv::Mat &leftImage, const cv::Mat &rightImage) const
    {
        // convert to cv from eigen
        cv::Mat K1, K2;
        std::vector<float> D1, D2;

        cv::eigen2cv(m_StereoCameraSetup.LeftCameraCalib.K, K1);
        K1.convertTo(K1, CV_64F);

        cv::eigen2cv(m_StereoCameraSetup.RightCameraCalib.K, K2);
        K2.convertTo(K2, CV_64F);

        cv::eigen2cv(m_StereoCameraSetup.LeftCameraCalib.D, D1);
        cv::eigen2cv(m_StereoCameraSetup.RightCameraCalib.D, D2);

        // other required matrices
        cv::Mat R1 = m_StereoCameraSetup.Rectification.RL;
        cv::Mat R2 = m_StereoCameraSetup.Rectification.RR;
        cv::Mat P1 = m_StereoCameraSetup.Rectification.PL;
        cv::Mat P2 = m_StereoCameraSetup.Rectification.PR;

        cv::Size size {m_StereoCameraSetup.LeftCameraCalib.ImageResolutionInPixels.x(), m_StereoCameraSetup.LeftCameraCalib.ImageResolutionInPixels.y() };

        // remap image using rectified projection
        cv::Mat map11, map12, map21, map22;
        initUndistortRectifyMap(K1, D1, R1, P1, size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(K2, D2, R2, P2, size, CV_16SC2, map21, map22);

        cv::Mat leftImageRectified, rightImageRectified;
        cv::remap(leftImage, leftImageRectified, map11, map12, cv::INTER_LINEAR);
        cv::remap(rightImage, rightImageRectified, map21, map22, cv::INTER_LINEAR);

        // debugging: write out to file
        cv::imwrite("left_rect.png", leftImageRectified);
        cv::imwrite("right_rect.png", rightImageRectified);

        // compute disparity
        cv::Mat disparity;
        m_StereoMatcher->compute(leftImageRectified, rightImageRectified, disparity);

        return disparity;
    }

    // Point cloud generation
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::GeneratePointCloud(const cv::Mat& disparity, const cv::Mat& cameraImage) const
    {
        cv::Mat reprojected3D;
        cv::reprojectImageTo3D(disparity, reprojected3D, m_StereoCameraSetup.Rectification.Q);

        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

        cv::Vec3f coords;
        pcl::PointXYZRGB point;

        uint32_t count = 0;

        for (int row = 0; row < reprojected3D.rows; row++)
        {
            for (int col = 0; col < reprojected3D.cols; col++)
            {
                coords = reprojected3D.at<cv::Vec3f>(row, col);

                // skip points with infinity values and zero depth
                if (isinf(coords[0]) || isinf(coords[1]) || isinf(coords[2]) || (coords[2] >= 5.90 && coords[2] < 6.0)) {
                    continue;
                }

                // set 3D point X,Y,Z and RGB
                point.x = coords[0];
                point.y = coords[1];
                point.z = coords[2];

                point.r = cameraImage.at<cv::Vec3b>(row, col)[2];
                point.g = cameraImage.at<cv::Vec3b>(row, col)[1];
                point.b = cameraImage.at<cv::Vec3b>(row, col)[0];

                pointCloud.push_back(point);

                count++;
            }
        }

        pointCloud.width = count;
        pointCloud.height = 1;
        pointCloud.is_dense = true;

        return pointCloud;
    }

    // Generate point cloud by direct calculation from disparity
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::Triangulate3D(const cv::Mat &disparity, const cv::Mat& leftCamImage, const cv::Mat& rightCamImage) const
    {
        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

        cv::Vec3f coords;
        pcl::PointXYZRGB point;

        uint32_t count = 0;
        short d;
        float f = m_StereoCameraSetup.LeftCameraCalib.K(0, 0);

        for (int i = 0; i < disparity.rows; i++)
        {
            for (int j = 0; j < disparity.cols; j++)
            {
                d = disparity.at<short>(i, j);
                if (d == -16 || d == 0) {
                    continue;
                }

                point.z = f * (m_StereoCameraSetup.T(0) / static_cast<float>(d));
                point.x = (static_cast<float>(i) - m_StereoCameraSetup.LeftCameraCalib.K(0, 2)) * (point.z / f);
                point.y = (static_cast<float>(j) - m_StereoCameraSetup.LeftCameraCalib.K(1, 2)) * (point.z / f);

                point.r = leftCamImage.at<cv::Vec3b>(i, j)[2];
                point.g = leftCamImage.at<cv::Vec3b>(i, j)[1];
                point.b = leftCamImage.at<cv::Vec3b>(i, j)[0];

                pointCloud.push_back(point);
                count++;
            }
        }

        pointCloud.width = count;
        pointCloud.height = 1;
        pointCloud.is_dense = true;

        return pointCloud;
    }
}
