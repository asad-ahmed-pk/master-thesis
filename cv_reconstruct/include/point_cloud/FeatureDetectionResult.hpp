//
// FeatureDetectionResult.hpp
// Holds the results of a feature detection based on a selected type of feature detector
//

#ifndef MASTER_THESIS_FEATUREDETECTIONRESULT_HPP
#define MASTER_THESIS_FEATUREDETECTIONRESULT_HPP

#include "point_cloud_constants.hpp"

namespace PointCloud
{
    struct FeatureDetectionResult
    {
        FeatureDetectorType type;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHFeatures;
        pcl::PointCloud<pcl::SHOT352>::Ptr SHOTFeatures;
    };
}

#endif //MASTER_THESIS_FEATUREDETECTIONRESULT_HPP
