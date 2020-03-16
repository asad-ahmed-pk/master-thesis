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
        m_Reconstructor = std::make_unique<Reconstruct::Reconstruct3D>(calib, config);

        // point cloud post processor
        m_PointCloudPostProcessor = std::make_unique<PointCloud::PointCloudPostProcessor>(m_Config);
        m_PointCloudPostProcessor->SetMinimumNeighboursOutlierRemoval(m_Config.PointCloudPostProcess.OutlierMinK);
        m_PointCloudPostProcessor->SetStdDevOutlierRemoval(m_Config.PointCloudPostProcess.OutlierStdDevThreshold);

        // localization
        m_Localizer = std::make_unique<Reconstruct::Localizer>();

        // point cloud registration
        m_PointCloudRegistration = std::make_unique<PointCloud::PointCloudRegistration>(config);
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

        Eigen::Vector3f t = m_Localizer->GetFrameWorldPose(frame).block(0, 3, 3, 1);

        if (frame.ID == 0)
        {
            ProcessFirstFrame(frame, pipelineResult);
            m_LastFramePosition = t;
            result = pipelineResult.PointCloudLocalized;
        }
        else
        {
            // only process if vehicle moved certain distance
            float distance = (t - m_LastFramePosition).norm();
            if (distance >= 5.0f)
            {
                std::cout << "\nProcessing frame #" << frame.ID << std::endl;
                ProcessSubsequentFrame(frame, pipelineResult);
                result = pipelineResult.PointCloudLocalized;

                m_LastFramePosition = t;
            }
        }

    }

    // Process this as the first frame
    void ReconstructionPipeline::ProcessFirstFrame(const Pipeline::StereoFrame &frame, PipelineResult& result)
    {
        // disparity image
        CalculateDisparity(frame, result.DisparityImage);

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr localPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(m_Reconstructor->Triangulate3D(result.DisparityImage, frame.LeftImage, frame.RightImage)));

        // remove outliers
        m_PointCloudPostProcessor->RemoveOutliers(localPointCloud, localPointCloud);

        // transform point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        Eigen::Matrix4f T = m_Localizer->TransformPointCloud(frame, *localPointCloud, *transformedPointCloud);

        // set first frame for registration pipeline
        cv::Mat projected3D;
        m_Reconstructor->Project3D(result.DisparityImage, projected3D);
        m_PointCloudRegistration->SaveFirstFrame(frame.LeftImage, projected3D, T);

        result.PointCloudLocalized = transformedPointCloud;

        *m_PrevPointCloud += *result.PointCloudLocalized;
    }

    // Process a frame that has had a frame processed before it. Assumes m_LastPipelineResult has valid data set.
    void ReconstructionPipeline::ProcessSubsequentFrame(const Pipeline::StereoFrame &frame, PipelineResult& result)
    {
        // disparity image
        CalculateDisparity(frame, result.DisparityImage);

        // triangulation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr localPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(m_Reconstructor->Triangulate3D(result.DisparityImage, frame.LeftImage, frame.RightImage)));

        // remove outliers
        m_PointCloudPostProcessor->RemoveOutliers(localPointCloud, localPointCloud);

        // transform point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        Eigen::Matrix4f T = m_Localizer->TransformPointCloud<pcl::PointXYZRGB>(frame, *localPointCloud, *transformedPointCloud);

        // align with last transformed point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        cv::Mat projected3D;
        m_Reconstructor->Project3D(result.DisparityImage, projected3D);
        m_PointCloudRegistration->RegisterFrameWithPreviousFrame(frame.LeftImage, projected3D, T, transformedPointCloud, alignedPointCloud);

        // ICP
        //m_PointCloudRegistration->AlignPointClouds(transformedPointCloud, m_PrevPointCloud);

        result.PointCloudLocalized = alignedPointCloud;

        m_PrevPointCloud->clear();
        *m_PrevPointCloud += *transformedPointCloud;
    }
}