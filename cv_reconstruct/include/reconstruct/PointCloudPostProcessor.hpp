//
// PointCloudPostProcessor.hpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#ifndef MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
#define MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
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

        /// Align the source point cloud onto the destination point cloud using ICP
        /// \param source The source point cloud being aligned
        /// \param target The point cloud that source should look like or approximate to
        /// \param result The result point cloud after the alignment is done
        /// \return True if a solution was found by ICP. Result is valid if returns true.
        bool AlignPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result);

        /// Set the minimum number of neighbours for outlier removal
        /// \param k The minimum number of k neighbours
        void SetMinimumNeighboursOutlierRemoval(int k);

        /// Set the standard deviation threshold to be considered an outlier for a point
        /// \param std The standard deviation scale factor
        void SetStdDevOutlierRemoval(double std);

    private:
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_OutlierRemover;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> m_ICP;
    };
}

#endif //MASTER_THESIS_POINTCLOUDPOSTPROCESSOR_HPP
