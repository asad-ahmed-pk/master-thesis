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
        m_PointCloudPostProcessor = std::make_unique<PointCloud::PointCloudPostProcessor>(m_Config);
        m_PointCloudPostProcessor->SetMinimumNeighboursOutlierRemoval(m_Config.PointCloudPostProcess.OutlierMinK);
        m_PointCloudPostProcessor->SetStdDevOutlierRemoval(m_Config.PointCloudPostProcess.OutlierStdDevThreshold);

        // localization
        m_Localizer = std::make_unique<Reconstruct::Localizer>();
    }

    // Calculate disparity map
    void ReconstructionPipeline::CalculateDisparity(const Pipeline::StereoFrame& frame, cv::Mat& disparity) const
    {
        // rectify if image rectification required
        if (m_ShouldRectifyImages) {
            cv::Mat leftImageRectified, rightImageRectified;
            m_Reconstructor->RectifyImages(frame.LeftImage, frame.RightImage, leftImageRectified, rightImageRectified);
            disparity = m_Reconstructor->GenerateDisparityMap(leftImageRectified, rightImageRectified);
        }
        else {
            disparity = m_Reconstructor->GenerateDisparityMap(frame.LeftImage, frame.RightImage);
        }
    }

    // Process frame and generate processed, localized, point cloud
    void ReconstructionPipeline::ProcessFrame(const StereoFrame& frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZRGB>() };
        PipelineResult pipelineResult{};

        if (frame.ID == 0) {
            ProcessFirstFrame(frame, pipelineResult);
        }
        else {
            ProcessSubsequentFrame(frame, pipelineResult);
        }

        result = pipelineResult.PointCloudLocalized;

        // store this frame's results for next frame to use (for alignment)
        m_LastFrameCameraImage = frame.LeftImage.clone();
        m_LastPipelineResult.DisparityImage = pipelineResult.DisparityImage.clone();
        pcl::copyPointCloud(*pipelineResult.PointCloudLocalized, *m_LastPipelineResult.PointCloudLocalized);
    }

    // Process this as the first frame
    void ReconstructionPipeline::ProcessFirstFrame(const Pipeline::StereoFrame &frame, PipelineResult& result)
    {
        // disparity image
        CalculateDisparity(frame, result.DisparityImage);

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr localPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(m_Reconstructor->GeneratePointCloud(result.DisparityImage, frame.LeftImage)));

        // remove outliers
        m_PointCloudPostProcessor->RemoveOutliers(localPointCloud, localPointCloud);

        // transform point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        m_Localizer->TransformPointCloud(frame, *localPointCloud, *transformedPointCloud);

        result.PointCloudLocalized = transformedPointCloud;
    }

    // Process a frame that has had a frame processed before it. Assumes m_LastPipelineResult has valid data set.
    void ReconstructionPipeline::ProcessSubsequentFrame(const Pipeline::StereoFrame &frame, PipelineResult& result)
    {
        // disparity image
        CalculateDisparity(frame, result.DisparityImage);

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr localPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(m_Reconstructor->GeneratePointCloud(result.DisparityImage, frame.LeftImage)));

        // remove outliers
        m_PointCloudPostProcessor->RemoveOutliers(localPointCloud, localPointCloud);

        // transform point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        m_Localizer->TransformPointCloud(frame, *localPointCloud, *transformedPointCloud);

        // attempt to align with last transformed point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        if (!m_PointCloudPostProcessor->AlignPointCloud(transformedPointCloud, m_LastPipelineResult.PointCloudLocalized, alignedPointCloud)) {
            //std::cout << "\nFailed to align point clouds!" << std::endl;
            result.PointCloudLocalized = transformedPointCloud;
        }
        else {
            result.PointCloudLocalized = alignedPointCloud;
        }
    }
}