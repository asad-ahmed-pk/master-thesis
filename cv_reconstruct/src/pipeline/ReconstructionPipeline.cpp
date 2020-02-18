//
// ReconstructionPipeline.cpp
// Pipeline responsible for complete processing of a frame to a point cloud
// Responsible for configuring required components according to the config
//

#include "pipeline/ReconstructionPipeline.hpp"

namespace Pipeline
{
    // Constructor
    ReconstructionPipeline::ReconstructionPipeline(const Config::Config& config, const Camera::Calib::StereoCalib& calib, bool isProcessingRectifiedImages) : m_ShouldRectifyImages(!isProcessingRectifiedImages), m_Config(config)
    {
        // setup pipeline components

        // 3D reconstructor
        m_Reconstructor = std::make_unique<Reconstruct::Reconstruct3D>(calib);
        m_Reconstructor->SetBlockMatcherType(m_Config.Reconstruction.BlockMatcherType);
        m_Reconstructor->SetStereoBMNumDisparities(m_Config.Reconstruction.NumDisparities);
        m_Reconstructor->SetStereoBMWindowSize(m_Config.Reconstruction.WindowSize);

        // point cloud post processor
        m_PointCloudPostProcessor = std::make_unique<Reconstruct::PointCloudPostProcessor>();
        m_PointCloudPostProcessor->SetMinimumNeighboursOutlierRemoval(m_Config.PointCloudPostProcess.OutlierMinK);
        m_PointCloudPostProcessor->SetStdDevOutlierRemoval(m_Config.PointCloudPostProcess.OutlierStdDevThreshold);

        // localization
        m_Localizer = std::make_unique<Reconstruct::Localizer>();
    }

    // Process frame and generate processed, localized, point cloud
    void ReconstructionPipeline::ProcessFrame(const StereoFrame& frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
    {
        cv::Mat disparity;

        // rectify if image rectification required
        if (m_ShouldRectifyImages) {
            cv::Mat leftImageRectified, rightImageRectified;
            m_Reconstructor->RectifyImages(frame.LeftImage, frame.RightImage, leftImageRectified, rightImageRectified);
            disparity = m_Reconstructor->GenerateDisparityMap(leftImageRectified, rightImageRectified);
        }
        else {
            disparity = m_Reconstructor->GenerateDisparityMap(frame.LeftImage, frame.RightImage);
        }

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>(std::move(m_Reconstructor->GeneratePointCloud(disparity, frame.LeftImage))));

        // remove outliers
        m_PointCloudPostProcessor->RemoveOutliers(temp, temp);

        // transform point cloud
        m_Localizer->TransformPointCloud(frame, *temp, *pointCloud);

        // attempt to align with last point cloud
        auto alignedPointCloudOutput = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (m_LastFramePointCloud != nullptr)
        {
           if (!m_PointCloudPostProcessor->AlignPointCloud(pointCloud, m_LastFramePointCloud, alignedPointCloudOutput)) {
               //std::cout << "\nFailed to align point clouds!" << std::endl;
           }
           else
           {
               //std::cout << "\nPoint cloud aligned to previous point cloud" << std::endl;
               pointCloud->clear();
               *pointCloud += *alignedPointCloudOutput;
           }
        }

        m_LastFramePointCloud = pointCloud;
    }
}