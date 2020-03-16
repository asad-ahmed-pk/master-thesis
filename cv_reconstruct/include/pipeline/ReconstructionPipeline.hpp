//
// ReconstructionPipeline.hpp
// Pipeline responsible for complete processing of a frame to a point cloud
// Responsible for configuring required components according to the config
//

#ifndef MASTER_THESIS_RECONSTRUCTIONPIPELINE_HPP
#define MASTER_THESIS_RECONSTRUCTIONPIPELINE_HPP

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "config/Config.hpp"
#include "StereoFrame.hpp"
#include "PipelineResult.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "point_cloud/PointCloudRegistration.hpp"
#include "point_cloud/PointCloudPostProcessor.hpp"

namespace Pipeline
{
    class ReconstructionPipeline
    {
    public:
        /// Create default instance of pipeline with the given config file
        /// \param config The config with params for reconstruction
        /// \param calib The stereo calibration for 3D reconstruction
        /// \param isProcessingRectifiedImages Set to true if this pipeline will process rectified images and will not attempt to rectify images
        ReconstructionPipeline(const Config::Config& config, const Camera::Calib::StereoCalib& calib, bool isProcessingRectifiedImages);

        ~ReconstructionPipeline() = default;

        /// Process the given stereo frame and compute the point cloud
        /// \param frame The stereo frame
        /// \param result Will be set to point to the computed 3D points in world space
        void ProcessFrame(const StereoFrame& frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result);

    private:
        void CalculateDisparity(const StereoFrame& frame, cv::Mat& disparity) const;
        void ProcessFirstFrame(const StereoFrame& frame, PipelineResult& result);
        void ProcessSubsequentFrame(const StereoFrame& frame, PipelineResult& result);

    private:
        Config::Config m_Config;
        bool m_ShouldRectifyImages;

        Eigen::Vector3f m_LastFramePosition;

        std::unique_ptr<Reconstruct::Reconstruct3D> m_Reconstructor;
        std::unique_ptr<Reconstruct::Localizer> m_Localizer;
        std::unique_ptr<PointCloud::PointCloudPostProcessor> m_PointCloudPostProcessor;
        std::unique_ptr<PointCloud::PointCloudRegistration> m_PointCloudRegistration;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_PrevPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
    };
}

#endif //MASTER_THESIS_RECONSTRUCTIONPIPELINE_HPP
