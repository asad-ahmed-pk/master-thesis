//
// PointCloudRegistration.hpp
// Handles point cloud registration
//

#ifndef MASTER_THESIS_POINTCLOUDREGISTRATION_HPP
#define MASTER_THESIS_POINTCLOUDREGISTRATION_HPP

#include <vector>
#include <memory>

#include <opencv2/core/types.hpp>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>

#include "pipeline/FrameFeatureExtractor.hpp"
#include "config/Config.hpp"
#include "point_cloud_constants.hpp"


namespace PointCloud
{
    class PointCloudRegistration
    {
    public:
        /// Create a default instance of the registration pipeline with the given config
        /// \param config The config with params set for ICP
        PointCloudRegistration(const Config::Config& config);

        ~PointCloudRegistration() = default;

        /// Register this frame with the previous frame and generate the registered point cloud
        /// \param image The RGB image to be used for 2D feature detection
        /// \param projected3D The 3D image of the frame which contains the X,Y,Z points in camera space
        /// \param worldTransform The world transform of that the computed point cloud should have
        /// \param input The input point cloud being aligned
        /// \param registeredCloud The pointer to be filled with the registered point cloud data
        void RegisterFrameWithPreviousFrame(const cv::Mat& image, const cv::Mat& projected3D, const Eigen::Matrix4f& worldTransform, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud);

        /// Save this as the first frame of the reconstruction process
        /// \param image The RGB image to be used for 2D feature detection
        /// \param projected3D The 3D image of the frame which contains the X,Y,Z points in camera space
        /// \param transform The 3D rigid-body transform to apply to the point cloud for this frame
        void SaveFirstFrame(const cv::Mat& image, const cv::Mat& projected3D, const Eigen::Matrix4f& transform);

        void AlignPointCloudWithPrevious(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result);

    private:
        void GeneratePointCloudsFrom2DCorrespondences(const std::vector<cv::KeyPoint>& sourceKeypoints2D, const cv::Mat& source3DReprojection,
                                                      const std::vector<cv::DMatch>& matches2D, const Eigen::Matrix4f& worldTransform,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud,
                                                      pcl::CorrespondencesPtr correspondences, float& maxDistance) const;

        Eigen::Matrix4f Estimate3DTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                pcl::CorrespondencesConstPtr correspondences, float maxDistance);

    private:
        struct TempData {
            cv::Mat Image;
            std::vector<cv::KeyPoint> Keypoints;
            cv::Mat Descriptors;
            cv::Mat Projected3DImage;
            Eigen::Matrix4f WorldTransform;
        };

    private:
        Pipeline::FrameFeatureExtractor m_2DFeatureExtractor;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_ICP;
        TempData m_TargetData;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_TargetCloud { nullptr };
    };
}

#endif //MASTER_THESIS_POINTCLOUDREGISTRATION_HPP
