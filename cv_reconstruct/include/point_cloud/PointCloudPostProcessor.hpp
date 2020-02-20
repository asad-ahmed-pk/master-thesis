//
// PointCloudPostProcessor.hpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#ifndef MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
#define MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/feature.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "point_cloud_constants.hpp"
#include "FeatureExtractor.hpp"
#include "config/Config.hpp"

namespace PointCloud
{
    class PointCloudPostProcessor
    {
    public:
        /// Create default instance of a point cloud post processor with the config
        /// \param config The config file with params set for point cloud processing
        PointCloudPostProcessor(const Config::Config& config);

        ~PointCloudPostProcessor() = default;

        /// Remove outliers using configured statistics
        /// \param input Point cloud pointer to input point cloud
        /// \param output Point cloud pointer to output point cloud
        void RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

        /// Align the source point cloud onto the destination point cloud using ICP
        /// \param source The source point cloud being aligned
        /// \param target The point cloud that source should look like or approximate to
        /// \param result The result point cloud after the alignment is done
        /// \return True if a solution was found by ICP. Result is valid if returns true.
        bool AlignPointCloud(PointCloudConstPtr source, PointCloudConstPtr target, PointCloudPtr result);

        /// Set the minimum number of neighbours for outlier removal
        /// \param k The minimum number of k neighbours
        void SetMinimumNeighboursOutlierRemoval(int k);

        /// Set the standard deviation threshold to be considered an outlier for a point
        /// \param std The standard deviation scale factor
        void SetStdDevOutlierRemoval(double std);

    private:
        void ExtractFeatures(PointCloudConstPtr cloud, FeatureDetectionResult& result) const;

    private:
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_OutlierRemover;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> m_ICP;

        pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr m_KeypointDetector;
        pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr m_FeatureDescriptor;

    private:
        std::unique_ptr<FeatureExtractor> m_FeatureExtractor;
        Config::Config m_Config;
    };
}

#endif //MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
