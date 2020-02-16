//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "reconstruct/Reconstruct3D.hpp"

#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>

#define MISSING_DISPARITY_Z 10000

namespace Reconstruct
{
    //const int SGM_MIN_DISPARITY = 1;
    //constexpr int SGM_P1 = 8 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;
    //constexpr int SGM_P2 = 32 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;

    // Constructor
    Reconstruct3D::Reconstruct3D(const Camera::Calib::StereoCalib& stereoSetup, bool shouldRectify) : m_StereoCameraSetup(stereoSetup), m_ShouldRectifyImages(shouldRectify)
    {
        // setup stereo matcher

        /*
        m_StereoMatcher = cv::StereoSGBM::create(SGM_MIN_DISPARITY, SGM_NUM_DISPARITIES, SGM_BLOCK_SIZE, SGM_P1, SGM_P2);

        m_StereoMatcher->setUniquenessRatio(10);
        m_StereoMatcher->setSpeckleRange(2);
        m_StereoMatcher->setSpeckleWindowSize(128);
        m_StereoMatcher->setDisp12MaxDiff(1);
        m_StereoMatcher->setPreFilterCap(10);
         */

        // set default block matcher
        SetBlockMatcherType(STEREO_BLOCK_MATCHER);
    }

    // Disparity map
    cv::Mat Reconstruct3D::GenerateDisparityMap(const cv::Mat &leftImage, const cv::Mat &rightImage) const
    {
        // compute disparity
        cv::Mat disparity;
        cv::Mat leftImageGrey, rightImageGrey;

        cv::cvtColor(leftImage, leftImageGrey, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightImage, rightImageGrey, cv::COLOR_BGR2GRAY);

        m_StereoMatcher->compute(leftImageGrey, rightImageGrey, disparity);

        return disparity;
    }

    // Point cloud generation
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::GeneratePointCloud(const cv::Mat& disparity, const cv::Mat& cameraImage) const
    {
        Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
        Q(2, 2) = 0.005 * m_StereoCameraSetup.LeftCameraCalib.K(0, 0);
        cv::Mat Q_CV(4, 4, CV_64F);
        cv::eigen2cv(Q, Q_CV);

        cv::Mat reprojected3D;
        cv::reprojectImageTo3D(disparity, reprojected3D, Q_CV, true);

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
                if (coords[2] == MISSING_DISPARITY_Z) {
                    continue;
                }

                // set 3D point X,Y,Z and RGB
                point.x = coords[0];
                point.y = -coords[1];
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
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::Triangulate3D(const cv::Mat &disparity, const cv::Mat& leftImage, const cv::Mat& rightImage) const
    {
        // get true disparity
        cv::Mat trueDisparity;
        disparity.convertTo(trueDisparity, CV_32F, 1.0 / 16.0, 0.0);

        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

        cv::Vec3f coords;
        pcl::PointXYZRGB point;

        uint32_t count = 0;
        float d;
        float f = (m_StereoCameraSetup.LeftCameraCalib.K(0, 0) + m_StereoCameraSetup.RightCameraCalib.K(1, 1)) / 2.0;
        //f *= 0.005;

        for (int i = 0; i < disparity.rows; i++)
        {
            for (int j = 0; j < disparity.cols; j++)
            {
                // skip zero disparities
                d = trueDisparity.at<float>(i, j);
                if (d <= 0.0) {
                    continue;
                }

                point.z = f * (m_StereoCameraSetup.T(0) / d);
                point.x = (static_cast<float>(i) - m_StereoCameraSetup.LeftCameraCalib.K(0, 2)) * (point.z / f);
                point.y = (static_cast<float>(j) - m_StereoCameraSetup.LeftCameraCalib.K(1, 2)) * (point.z / f);

                point.r = leftImage.at<cv::Vec3b>(i, j)[2];
                point.g = leftImage.at<cv::Vec3b>(i, j)[1];
                point.b = leftImage.at<cv::Vec3b>(i, j)[0];

                pointCloud.push_back(point);
                count++;
            }
        }

        pointCloud.width = count;
        pointCloud.height = 1;
        pointCloud.is_dense = true;

        return pointCloud;
    }

    // Apply stereo rectification to left and right images
    void Reconstruct3D::RectifyImages(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& rectLeftImage, cv::Mat& rectRightImage) const
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

        cv::Size size(leftImage.cols, rightImage.rows);

        // remap image using rectified projection
        cv::Mat map11, map12, map21, map22;
        initUndistortRectifyMap(K1, D1, R1, P1, size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(K2, D2, R2, P2, size, CV_16SC2, map21, map22);

        cv::Mat leftImageRectified, rightImageRectified;
        cv::remap(leftImage, rectLeftImage, map11, map12, cv::INTER_LINEAR);
        cv::remap(rightImage, rectRightImage, map21, map22, cv::INTER_LINEAR);
    }

    // TODO: Fix - compute 3D locations directly from parallax map using matrix operation
    // useful later for parallel implementation
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::PointCloudMatrixCompute(const cv::Mat &leftImage, const cv::Mat &disparity) const
    {
        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
        pcl::PointXYZRGB point;

        uint32_t count = 0;
        float f = m_StereoCameraSetup.LeftCameraCalib.K(0, 0);          // focal length
        float b = m_StereoCameraSetup.T(0);                                 // baseline

        // new method using matrix multiplication
        int N = disparity.rows * disparity.cols;

        // Matrix P: will be filled with 3D coordinates X, Y, Z, W
        cv::Mat P(4, N, CV_32FC1);

        // Matrix M: Image scale matrix
        cv::Mat M = cv::Mat::zeros(4, 4, CV_32FC1);

        // Matrix I: Image matrix with image points (4xN)
        cv::Mat I(4, N, CV_32FC1, cv::Scalar(1.0));
        int n = 0;
        for (int y = 0; y < disparity.rows; y++)
        {
            for (int x = 0; x < disparity.cols; x++)
            {
                I.at<float>(0, n) = x;
                I.at<float>(1, n) = y;
                I.at<float>(3, n) = static_cast<float>(disparity.at<short>(x, y));
                n++;
            }
        }

        // calculate by matrix multiplication
        P = M * I;

        // create point cloud from result P matrix
        int row, col = 0;
        for (int n = 0; n < N; n++)
        {
            point.x = P.at<float>(0, n) / P.at<float>(3, n);
            point.y = P.at<float>(1, n) / P.at<float>(3, n);
            point.z = P.at<float>(2, n) / P.at<float>(3, n);

            col = static_cast<int>(I.at<float>(0, n));
            row = static_cast<int>(I.at<float>(1, n));

            point.r = leftImage.at<cv::Vec3b>(row, col)[2];
            point.g = leftImage.at<cv::Vec3b>(row, col)[1];
            point.b = leftImage.at<cv::Vec3b>(row, col)[0];

            pointCloud.push_back(point);
            count++;
        }

        pointCloud.width = count;
        pointCloud.height = 1;
        pointCloud.is_dense = true;

        return pointCloud;
    }

    // Process frame
    void Reconstruct3D::ProcessFrame(const Reconstruct::StereoFrame& frame, pcl::PointCloud<pcl::PointXYZRGB>& pointCloud)
    {
        cv::Mat disparity;

        // rectify if image rectification required
        if (m_ShouldRectifyImages) {
            cv::Mat leftImageRectified, rightImageRectified;
            RectifyImages(frame.LeftImage, frame.RightImage, leftImageRectified, rightImageRectified);
            disparity = GenerateDisparityMap(leftImageRectified, rightImageRectified);
        }
        else {
            disparity = GenerateDisparityMap(frame.LeftImage, frame.RightImage);
        }

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB> temp = GeneratePointCloud(disparity, frame.LeftImage);
        //pcl::PointCloud<pcl::PointXYZRGB> temp = Triangulate3D(disparity, frame.LeftImage, frame.RightImage);

        // transform point cloud
        m_Localizer.TransformPointCloud(frame, temp, pointCloud);
    }

    void Reconstruct3D::SetStereoBMWindowSize(int size) {
        m_StereoMatcher->setBlockSize(size);
    }

    void Reconstruct3D::SetStereoBMNumDisparities(int num) {
        m_StereoMatcher->setNumDisparities(num);
    }

    void Reconstruct3D::SetBlockMatcherType(StereoBlockMatcherType type)
    {
        if (m_StereoMatcher != nullptr) {
            m_StereoMatcher = nullptr;
        }

        // create block matchers with default args
        switch (type)
        {
            case STEREO_BLOCK_MATCHER:
                m_StereoMatcher = cv::StereoBM::create(16, 21);
                break;

            case STEREO_SEMI_GLOBAL_BLOCK_MATCHER:
                m_StereoMatcher = cv::StereoSGBM::create(0, 16, 3);
        }
    }
}
