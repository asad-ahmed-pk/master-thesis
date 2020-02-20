//
// FeatureExtractor.cpp
// Extracts features and keypoints from point clouds
// Configurable to use different keypoints and feature extractors (defined in point_cloud_constants.hpp)
//

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>

#include "point_cloud/FeatureExtractor.hpp"

namespace PointCloud
{
    FeatureExtractor::FeatureExtractor(KeypointType keypointDetector, FeatureDetectorType featureDetector, const Config::Config& config) : m_KeypointDetectorType(keypointDetector), m_FeatureDetectorType(featureDetector), m_Config(std::move(config))
    {
        // setup the keypoint based on type
        switch (keypointDetector)
        {
            case KEYPOINT_SIFT:
                m_KeypointDetector = pcl::SIFTKeypoint<PointType, pcl::PointWithScale>::Ptr(new pcl::SIFTKeypoint<PointType, pcl::PointWithScale>());
                break;

            case KEYPOINT_ISS_3D:
                m_KeypointDetector = pcl::ISSKeypoint3D<PointType, PointType, pcl::Normal>::Ptr(new pcl::ISSKeypoint3D<PointType, PointType, pcl::Normal>());
                break;
        }

        // setup feature detector based on the type
        switch (featureDetector)
        {
            case FEATURE_DETECTOR_FPFH:
                m_FeatureDetector = pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr(new pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>());
                break;

            case FEATURE_DETECTOR_SHOT_COLOR:
                m_FeatureDetector = pcl::SHOTColorEstimation<PointType, pcl::Normal, pcl::SHOT1344>::Ptr(new pcl::SHOTColorEstimation<PointType, pcl::Normal, pcl::SHOT1344>());
                break;
        }

        // configure the detectors
        ConfigureKeypointDetector();
        ConfigureFeatureDetector();
    }

    void FeatureExtractor::ConfigureKeypointDetector()
    {
        // TODO: configure using m_Config based on the type
        switch (m_KeypointDetectorType)
        {
            case KEYPOINT_SIFT:
            {
                auto sift = boost::static_pointer_cast<pcl::SIFTKeypoint<PointType, pcl::PointWithScale>>(m_KeypointDetector);
                sift->setScales(m_Config.PointCloudKeypointDetection.SIFT.MinScale, m_Config.PointCloudKeypointDetection.SIFT.NumOctaves, m_Config.PointCloudKeypointDetection.SIFT.NumScalesPerOctave);
                sift->setMinimumContrast(m_Config.PointCloudKeypointDetection.SIFT.MinContrast);
                break;
            }

            case KEYPOINT_ISS_3D:
            {
                // TODO: configure ISS3D
                break;
            }
        }
    }

    void FeatureExtractor::ConfigureFeatureDetector()
    {
        switch (m_FeatureDetectorType)
        {
            case FEATURE_DETECTOR_FPFH:
            {
                auto fpfh = boost::static_pointer_cast<pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>>(m_FeatureDetector);
                fpfh->setRadiusSearch(m_Config.PointCloudFeatureDetection.FPFH.MinRadius);
                break;
            }

            case FEATURE_DETECTOR_SHOT_COLOR:
            {
                // TODO: configure SHOT feature detector
                break;
            }
        }
    }

    // Compute keypoints based on type
    void FeatureExtractor::ComputeKeypoints(PointCloudConstPtr cloud, NormalsPtr normals, KeypointDetectionResult& detectedKeypoints) const
    {
        // keypoints will be copied here from the specific keypoint detector output
        detectedKeypoints.Keypoints = PointCloudPtr(new pcl::PointCloud<PointType>());

        switch (m_KeypointDetectorType)
        {
            case KEYPOINT_SIFT:
            {
                auto sift = boost::static_pointer_cast<pcl::SIFTKeypoint<PointType, pcl::PointWithScale>>(m_KeypointDetector);
                pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());

                detectedKeypoints.SIFTKeypoints = pcl::PointCloud<pcl::PointWithScale>::Ptr(new pcl::PointCloud<pcl::PointWithScale>());

                sift->setSearchMethod(tree);
                sift->setSearchSurface(cloud);
                sift->setInputCloud(cloud);
                sift->compute(*detectedKeypoints.SIFTKeypoints);

                pcl::copyPointCloud(*detectedKeypoints.SIFTKeypoints, *detectedKeypoints.Keypoints);

                break;
            }
            case KEYPOINT_ISS_3D:
            {
                // TODO: Implement ISS_3D
                break;
            }
        }
    }

    // Compute features based on type
    void FeatureExtractor::ComputeFeatures(PointCloudConstPtr keypoints, NormalsPtr normals, FeatureDetectionResult& detectedFeatures) const
    {
        switch (m_FeatureDetectorType)
        {
            case FEATURE_DETECTOR_FPFH:
                ComputeFPFHFeatures(keypoints, normals, detectedFeatures);
                break;

            case FEATURE_DETECTOR_SHOT_COLOR:
                ComputeSHOTColorFeatures(keypoints, normals, detectedFeatures);
                break;
        }
    }

    // Compute normals
    void FeatureExtractor::ComputeNormals(PointCloudConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr computedNormals) const
    {
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

        normalEstimation.setSearchMethod(tree);
        normalEstimation.setRadiusSearch(m_Config.PointCloudFeatureDetection.Normals.Radius);
        normalEstimation.compute(*computedNormals);
    }

    // SHOT colour features
    void FeatureExtractor::ComputeSHOTColorFeatures(PointCloudConstPtr keypoints, NormalsPtr normals, FeatureDetectionResult& detectedFeatures) const
    {
        // TODO: Compute SHOT color features
    }

    // FPFH features
    void FeatureExtractor::ComputeFPFHFeatures(PointCloudConstPtr keypoints, NormalsPtr normals, FeatureDetectionResult& detectedFeatures) const
    {
        auto fpfh = boost::static_pointer_cast<pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>>(m_FeatureDetector);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

        detectedFeatures.Type = FEATURE_DETECTOR_FPFH;
        detectedFeatures.FPFHFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>());

        fpfh->setInputNormals(normals);
        fpfh->setInputCloud(keypoints);
        fpfh->setSearchMethod(tree);
        fpfh->compute(*detectedFeatures.FPFHFeatures);
    }
}