//
// PointCloudPostProcessor.cpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "reconstruct/PointCloudPostProcessor.hpp"

namespace Reconstruct
{
    PointCloudPostProcessor::PointCloudPostProcessor()
    {
        // setup outlier remover with default values
        m_OutlierRemover.setMeanK(50);
        m_OutlierRemover.setStddevMulThresh(1.0);

        // setup ICP alignment
        m_ICP.setMaximumIterations(25);
        m_ICP.setRANSACIterations(25);
        m_ICP.setMaxCorrespondenceDistance(100);
    }

    // Outlier removal
    void PointCloudPostProcessor::RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
    {
        m_OutlierRemover.setInputCloud(input);
        m_OutlierRemover.filter(*output);
    }

    // ICP alignment
    bool PointCloudPostProcessor::AlignPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result)
    {
        m_ICP.setInputSource(source);
        m_ICP.setInputTarget(target);
        m_ICP.align(*result);

        return m_ICP.hasConverged();
    }

    // Setters
    void PointCloudPostProcessor::SetMinimumNeighboursOutlierRemoval(int k) {
        m_OutlierRemover.setMeanK(k);
    }

    void PointCloudPostProcessor::SetStdDevOutlierRemoval(double std) {
        m_OutlierRemover.setStddevMulThresh(std);
    }
}
