//
// FeatureExtractionResult.hpp
// Contains structs that hold feature extraction results
//

#ifndef MASTER_THESIS_FEATUREEXTRACTIONRESULT_HPP
#define MASTER_THESIS_FEATUREEXTRACTIONRESULT_HPP

#include "point_cloud_constants.hpp"

namespace PointCloud
{
    struct FeatureDetectionResult
    {
        KeypointType type;
        pcl::PointCloud<pcl::FPFHSignature33> FPFHFeatures;
        pcl::PointCloud<pcl::SHOT352> SHOTFeatures;
    };

}

#endif //MASTER_THESIS_FEATUREEXTRACTIONRESULT_HPP
