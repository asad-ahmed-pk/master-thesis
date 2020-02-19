//
// FeatureExtractor.hpp
// Extracts features and keypoints from point clouds
// Configurable to use different keypoints and feature extractors (defined in point_cloud_constants.hpp)
//

#ifndef MASTER_THESIS_FEATUREEXTRACTOR_HPP
#define MASTER_THESIS_FEATUREEXTRACTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/keypoints/keypoint.h>

#include "FeatureExtractionResult.hpp"
#include "FeatureExtractionConfig.hpp"
#include "point_cloud_constants.hpp"

namespace PointCloud
{
    class FeatureExtractor
    {
    public:
        /// Construct an instance of a feature extractor for the given keypoint extractor and detector
        /// \param keypointDetector The type of keypoint detector to be used
        /// \param featureDetector The type of the feature detector to be used
        /// \param config The feature extraction config with params for the selected keypoint and feature detector set
        FeatureExtractor(KeypointType keypointDetector, FeatureDetectorType featureDetector, FeatureExtractionConfig config);

        ~FeatureExtractor() = default;

        /// Compute the keypoints for the point cloud
        /// \param cloud The point cloud for which keypoints will be computed
        /// \param computedKeypoints The point cloud that will be computed with keypoints
        void ComputeKeypoints(PointCloudPtr cloud, PointCloudPtr computedKeypoints) const;

        /// Detect the features for the point cloud keypoints
        /// \param keypoints The keypoints of the point cloud
        /// \param detectedFeatures The detection result. The appropriate field will contain the result
        /// depending on the feature detector type this extractor was configured with
        void ComputeFeatures(PointCloudPtr keypoints, FeatureDetectionResult detectedFeatures) const;

        /// Compute the normals for the given point cloud
        /// \param cloud The point cloud for which normals are to be computed
        /// \param computedNormals Will be populated with computed normals
        void ComputeNormals(PointCloudPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr computedNormals) const;

    private:
        void ConfigureKeypointDetector();
        void ConfigureFeatureDetector();

    private:
        KeypointType m_KeypointDetectorType;
        FeatureDetectorType m_FeatureDetectorType;

        FeatureExtractionConfig m_Config;

        pcl::Keypoint<PointType, PointType>::Ptr m_KeypointDetector;
        boost::shared_ptr<void> m_FeatureDetector;
    };
}

#endif //MASTER_THESIS_FEATUREEXTRACTOR_HPP
