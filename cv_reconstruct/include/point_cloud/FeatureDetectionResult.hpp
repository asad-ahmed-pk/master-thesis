//
// FeatureDetectionResult.hpp
// Holds the results of a feature detection based on a selected type of feature detector / keypoint detector
//

#ifndef MASTER_THESIS_FEATUREDETECTIONRESULT_HPP
#define MASTER_THESIS_FEATUREDETECTIONRESULT_HPP

#include "point_cloud_constants.hpp"

namespace PointCloud
{
    /// The result of extracting feature descriptors
    struct FeatureDetectionResult
    {
        FeatureDetectorType Type;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHFeatures;
        pcl::PointCloud<pcl::SHOT352>::Ptr SHOTFeatures;
    };

    /// The result of extracting keypoints
    struct KeypointDetectionResult
    {
        KeypointType Type;
        PointCloudPtr Keypoints;
        pcl::PointCloud<pcl::PointWithScale>::Ptr SIFTKeypoints;
    };
}

#endif //MASTER_THESIS_FEATUREDETECTIONRESULT_HPP
