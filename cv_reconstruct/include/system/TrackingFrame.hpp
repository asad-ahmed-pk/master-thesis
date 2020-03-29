//
// TrackingFrame.hpp
// Represents a frame used for tracking
// Holds information on colour space, disparity, and 2D features
//

#ifndef MASTER_THESIS_TRACKINGFRAME_HPP
#define MASTER_THESIS_TRACKINGFRAME_HPP

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include "reconstruct/Reconstruct3D.hpp"
#include "pipeline/FrameFeatureExtractor.hpp"

namespace System
{
    class TrackingFrame
    {
    public:
        /// Construct an instance from the given stereo frame
        /// \param leftImage The camera image (RGB)
        /// \param disparity The disparity image used for depth estimation
        /// \param featureExtractor Shared ptr to a 2D feature extractor
        /// \param reconstructor Shared ptr to a set-up 3D reconstructor
        TrackingFrame(const cv::Mat& cameraImage, const cv::Mat& disparity, std::shared_ptr<Pipeline::FrameFeatureExtractor> featureExtractor, std::shared_ptr<Reconstruct::Reconstruct3D> reconstructor);

        ~TrackingFrame() = default;
        
        size_t GetID() const;

        // Get features
        cv::Mat GetFeatureDescriptors() const;

        // Get keypoints
        std::vector<cv::KeyPoint> GetKeypoints() const;

        // Get keypoint point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetKeypointPointCloud() const;
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetDensePointCloud() const;
        
        cv::Mat GetCameraImageMask() const;
        
        cv::Mat GetCameraImage() const;
        
        cv::Mat GetDisparity() const;
        
        void SetTrackedPose(const Eigen::Isometry3d& pose);
        
        void SetID(size_t id);

    private:
        void PruneDisparityImage(cv::Mat& disparity, cv::Mat& mask) const;
        void SetupFrame();
        void ComputeKeypointCloud();
        void PruneFeatures();

    private:
        std::shared_ptr<Reconstruct::Reconstruct3D> m_3DReconstructor;
        std::shared_ptr<Pipeline::FrameFeatureExtractor> m_FeatureExtractor;

    private:
        size_t m_ID { 0 };
        cv::Mat m_CameraImage;
        cv::Mat m_Disparity;
        cv::Mat m_Descriptors;
        cv::Mat m_Mask;
        std::vector<cv::KeyPoint> m_Keypoints;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_KeypointPointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        
    private:
        Eigen::Isometry3d m_Pose;
    };
}

#endif //MASTER_THESIS_TRACKINGFRAME_HPP
