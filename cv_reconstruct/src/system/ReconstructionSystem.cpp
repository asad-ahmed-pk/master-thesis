//
// ReconstructionSystem.cpp
// Main system for 3D reconstruction. Computes keyframes, and creates 3D map.
//

#include <opencv2/imgproc/imgproc.hpp>

#include "system/ReconstructionSystem.hpp"

namespace System
{
    // Constructor
    ReconstructionSystem::ReconstructionSystem(const Config::Config& config, const Camera::Calib::StereoCalib& stereoCalib) : m_Config(config),
    m_3DReconstructor(stereoCalib, config), m_PointCloudRegistration(config)
    {
        m_MappingSystem.StartOptimisationThread();
    }

    // Process stereo frame
    void ReconstructionSystem::ProcessStereoFrame(const Pipeline::StereoFrame& stereoFrame)
    {
        // disparity image (used as basis for 3D reconstruction)
        cv::Mat disparity;
        cv::Mat leftImage; cv::Mat rightImage;

        // check if stereo rectification is needed (from config)
        if (m_Config.Reconstruction.ShouldRectifyImages) {
            m_3DReconstructor.RectifyImages(stereoFrame.LeftImage, stereoFrame.RightImage, leftImage, rightImage);
            disparity = m_3DReconstructor.GenerateDisparityMap(leftImage, rightImage);
        }
        else {
            leftImage = stereoFrame.LeftImage;
            rightImage = stereoFrame.RightImage;
        }

        // create disparity image from stereo frame
        disparity = m_3DReconstructor.GenerateDisparityMap(leftImage, rightImage);

        // prune disparity to remove values with higher uncertainty
        PruneDisparityImage(disparity);

        // triangulate and get 3D points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB>(std::move(m_3DReconstructor.Triangulate3D(disparity, leftImage, rightImage))) };

        // align to previous point cloud using ICP and get the estimated transform
        if (m_PreviousPointCloud != nullptr)
        {
            pcl::transformPointCloud(*cloud, *cloud, m_PreviousEstimatedPose);
            Eigen::Matrix4f T = m_PointCloudRegistration.RegisterCloudICP(cloud, m_PreviousPointCloud);
            m_PreviousEstimatedPose = T;
        }
        else {
             m_PreviousPointCloud = cloud;
        }

        // add to mapping system
        m_MappingSystem.AddPointCloud(*cloud);
    }

    // Prune disparity image
    void ReconstructionSystem::PruneDisparityImage(cv::Mat& disparity) const
    {
        // compute std dev of disparity and threshold it
        cv::Mat disp; cv::Mat mean; std::vector<double> std;
        cv::normalize(disparity, disp, 0, 255, cv::NORM_MINMAX, CV_8U);

        cv::Mat mask(disp.rows, disp.cols, CV_8U);
        cv::meanStdDev(disp, mean, std);

        cv::threshold(disp, mask, std[0] * 1.3, 255, cv::THRESH_BINARY);

        // apply mask to disparity and prune
        for (int row = 0; row < mask.rows; row++)
        {
            for (int col = 0; col < mask.cols; col++)
            {
                int value = static_cast<int>(mask.at<unsigned char>(row, col));
                if (value == 0) {
                    disparity.at<short>(row, col) = 0;
                }
            }
        }
    }

    // Shutdown request
    void ReconstructionSystem::RequestShutdown() {
        m_RequestedShutdown = true;
    }
}