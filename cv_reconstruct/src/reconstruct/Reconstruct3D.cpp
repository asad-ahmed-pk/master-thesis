//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "reconstruct/Reconstruct3D.hpp"
#include "reconstruct/Reconstruct3DTypes.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/stereo/disparity_map_converter.h>
#include <pcl/common/transforms.h>

#include <fstream>

#define MISSING_DISPARITY_Z 10000

namespace Reconstruct
{
    const int SGM_MIN_DISPARITY = 0;
    //constexpr int SGM_P1 = 8 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;
    //constexpr int SGM_P2 = 32 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;

    // Constructor
    Reconstruct3D::Reconstruct3D(const Camera::Calib::StereoCalib& stereoSetup, const Config::Config& config) : m_StereoCameraSetup(stereoSetup) {
        // setup stereo matcher
        ConfigureSteoreoMatcher(config);
    }

    // Configure stereo matcher from config
    void Reconstruct3D::ConfigureSteoreoMatcher(const Config::Config& config)
    {
        switch (config.Reconstruction.BlockMatcherType)
        {
            case STEREO_BLOCK_MATCHER: {
                SetBlockMatcherType(STEREO_BLOCK_MATCHER);
                m_StereoMatcher->setBlockSize(config.Reconstruction.SBM.WindowSize);
                m_StereoMatcher->setNumDisparities(config.Reconstruction.SBM.NumDisparities);
                break;
            }

            case STEREO_SEMI_GLOBAL_BLOCK_MATCHER: {
                SetBlockMatcherType(STEREO_SEMI_GLOBAL_BLOCK_MATCHER);
                auto sgbm = std::static_pointer_cast<cv::StereoSGBM>(m_StereoMatcher);

                sgbm->setMinDisparity(config.Reconstruction.SGBM.MinDisparity);
                sgbm->setPreFilterCap(config.Reconstruction.SGBM.PreFilterCap);
                sgbm->setBlockSize(config.Reconstruction.SGBM.BlockSize);
                sgbm->setNumDisparities(config.Reconstruction.SGBM.NumDisparities);
                sgbm->setSpeckleRange(config.Reconstruction.SGBM.SpeckleRange);
                sgbm->setSpeckleWindowSize(config.Reconstruction.SGBM.SpeckleWindowSize);
                sgbm->setUniquenessRatio(config.Reconstruction.SGBM.UniquenessRatio);

                // determine P1 and P2 coefficients
                sgbm->setP1(8 * 3 * sgbm->getBlockSize() * sgbm->getBlockSize());
                sgbm->setP2(32 * 3 * sgbm->getBlockSize() * sgbm->getBlockSize());

                break;
            }
        }
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

    // Get camera intrinsics
    void Reconstruct3D::GetCameraParameters(float& fx, float& fy, float& cx, float& cy, int camNumber) const
    {
        if (camNumber == 1)
        {
            fx = m_StereoCameraSetup.RightCameraCalib.K(0, 0);
            fy = m_StereoCameraSetup.RightCameraCalib.K(1, 1);
            cx = m_StereoCameraSetup.RightCameraCalib.K(0, 2);
            cy = m_StereoCameraSetup.RightCameraCalib.K(1, 2);
        }
        else
        {
            fx = m_StereoCameraSetup.LeftCameraCalib.K(0, 0);
            fy = m_StereoCameraSetup.LeftCameraCalib.K(1, 1);
            cx = m_StereoCameraSetup.LeftCameraCalib.K(0, 2);
            cy = m_StereoCameraSetup.LeftCameraCalib.K(1, 2);
        }
    }

    // Generate point cloud by direct calculation from disparity
    pcl::PointCloud<pcl::PointXYZRGB> Reconstruct3D::Triangulate3D(const cv::Mat& disparity, const cv::Mat& cameraImage, cv::InputArray& mask) const
    {
        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

        cv::Vec3f coords;
        pcl::PointXYZRGB point;

        uint32_t count = 0;
        float d;

        // baseline and focal length
        float b = m_StereoCameraSetup.T(0);
        float f = (m_StereoCameraSetup.LeftCameraCalib.K(0, 0) + m_StereoCameraSetup.LeftCameraCalib.K(1, 1)) / 2.0f;

        // principal point
        float cx = m_StereoCameraSetup.LeftCameraCalib.K(0, 2);
        float cy = m_StereoCameraSetup.LeftCameraCalib.K(1, 2);

        bool applyingMask = (&mask != &cv::noArray());
        cv::Mat maskImage;
        if (applyingMask) {
            maskImage = mask.getMat();
        }

        for (int i = 0; i < disparity.rows; i++)
        {
            for (int j = 0; j < disparity.cols; j++)
            {
                // skip zero disparities
                d = disparity.at<float>(i, j);
                if (d <= 0.0) {
                    continue;
                }

                // check for mask and skip triangulation if mask is being applied
                if (applyingMask) {
                    int value = static_cast<int>(maskImage.at<unsigned char>(i, j));
                    if (value == 0) {
                        continue;
                    }
                }

                point.z = f * b / d;
                point.x = (static_cast<float>(j) - cx) * (point.z / f);
                point.y = -(static_cast<float>(i) - cy) * (point.z / f);

                point.r = cameraImage.at<cv::Vec3b>(i, j)[2];
                point.g = cameraImage.at<cv::Vec3b>(i, j)[1];
                point.b = cameraImage.at<cv::Vec3b>(i, j)[0];

                pointCloud.push_back(point);
                count++;
            }
        }

        pointCloud.width = count;
        pointCloud.height = 1;
        pointCloud.is_dense = true;

        return pointCloud;
    }

    // Triangulate vector of 2D points using disparity
    void Reconstruct3D::TriangulatePoints(const cv::Mat& disparity, const cv::Mat& cameraImage, const std::vector<cv::KeyPoint>& points, std::vector<pcl::PointXYZRGB>& triangulatedPoints) const
    {
        // get cam params
        float fx, fy, cx, cy, b;
        GetCameraParameters(fx, fy, cx, cy);
        b = m_StereoCameraSetup.T(0);
        
        pcl::PointXYZRGB P;
        cv::Vec3b color;
        float d = 0.0;
        
        // triangulate each 2D point to 3D
        for (const cv::KeyPoint& p : points)
        {
            d = disparity.at<float>(static_cast<int>(p.pt.y + 0.5), static_cast<int>(p.pt.x + 0.5));
            color = cameraImage.at<cv::Vec3b>(p.pt.y, p.pt.x);
            
            // no disparity for this point!
            if (d <= 0.0) {
                std::cerr << "\nERROR: Negative or 0 disparity!" << d << std::endl;
            }
            
            P = pcl::PointXYZRGB(color[2], color[1], color[0]);
            
            P.z = fx * b / d;
            P.x = (p.pt.x - cx) * P.z / fx;
            P.y = -(p.pt.y - cy) * P.z / fy;
            
            triangulatedPoints.push_back(P);
        }
    }

    // Back projection
    pcl::PointXYZ Reconstruct3D::BackProjectPoint(float x, float y) const
    {
        float fx = m_StereoCameraSetup.LeftCameraCalib.K(0, 0);
        float fy = m_StereoCameraSetup.LeftCameraCalib.K(1, 1);
        float cx = m_StereoCameraSetup.LeftCameraCalib.K(0, 2);
        float cy = m_StereoCameraSetup.LeftCameraCalib.K(1, 2);
        
        float Z = 1.0f;
        float X = (x - cx) * Z / fx;
        float Y = (y - cy) * Z / fy;
        
        pcl::PointXYZ p;
        p.x = X;
        p.y = Y;
        p.z = Z;
        
        return p;
    }

    // Triangulate a single image point to 3D space
    pcl::PointXYZRGB Reconstruct3D::TriangulateUV(float u, float v, float disparity, const cv::Vec3b& color) const
    {
        // baseline and focal length
        float b = m_StereoCameraSetup.T(0);
        float f = (m_StereoCameraSetup.LeftCameraCalib.K(0, 0) + m_StereoCameraSetup.LeftCameraCalib.K(1, 1)) / 2.0f;

        // principal point
        float cx = m_StereoCameraSetup.LeftCameraCalib.K(0, 2);
        float cy = m_StereoCameraSetup.LeftCameraCalib.K(1, 2);
        
        pcl::PointXYZRGB point;
        
        point.z = f * b / disparity;
        point.x = (static_cast<float>(u) - cx) * (point.z / f);
        point.y = -(static_cast<float>(v) - cy) * (point.z / f);

        point.r = color[2];
        point.g = color[1];
        point.b = color[0];
        
        return point;
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

    // Setters

    void Reconstruct3D::SetStereoBMWindowSize(int size) {
        m_StereoMatcher->setBlockSize(size);
    }

    void Reconstruct3D::SetStereoBMNumDisparities(int num) {
        m_StereoMatcher->setNumDisparities(num);
    }

    void Reconstruct3D::SetBlockMatcherType(StereoBlockMatcherType type)
    {
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
