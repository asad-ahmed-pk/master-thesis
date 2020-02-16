//
// PointCloudPostProcessor.hpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#ifndef MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
#define MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace Reconstruct
{
    class PointCloudPostProcessor
    {
    public:
        /// Create default instance of a point cloud post processor
        PointCloudPostProcessor();

        ~PointCloudPostProcessor() = default;

        /// Remove outliers using configured statistics
        /// \param input Point cloud pointer to input point cloud
        /// \param output Point cloud pointer to output point cloud
        void RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

        /// Set the minimum number of neighbours for outlier removal
        /// \param k The minimum number of k neighbours
        void SetMinimumNeighboursOutlierRemoval(int k);

        /// Set the standard deviation threshold to be considered an outlier for a point
        /// \param std
        void SetStdDevOutlierRemoval(double std);

    private:
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_OutlierRemover;
    };
}

#endif //MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
