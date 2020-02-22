//
// PointCloudPostProcessor.cpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/io/pcd_io.h>

#include "point_cloud/PointCloudPostProcessor.hpp"
#include "point_cloud/FeatureExtractor.hpp"

typedef pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> FPFH;

namespace PointCloud
{
    PointCloudPostProcessor::PointCloudPostProcessor(const Config::Config& config) : m_Config(config)
    {
        // setup outlier remover with default values
        m_OutlierRemover.setMeanK(m_Config.PointCloudPostProcess.OutlierMinK);
        m_OutlierRemover.setStddevMulThresh(m_Config.PointCloudPostProcess.OutlierStdDevThreshold);

        // setup ICP alignment
        m_ICP.setMaximumIterations(25);
        m_ICP.setTransformationEpsilon (1e-8);
        m_ICP.setEuclideanFitnessEpsilon (1);
        m_ICP.setRANSACIterations(25);
        m_ICP.setMaxCorrespondenceDistance(m_Config.PointCloudFeatureDetection.FPFH.MinRadius);

        // setup feature extractor (finds both keypoints and feature descriptors for those keypoints)
        m_FeatureExtractor = std::make_unique<FeatureExtractor>(m_Config.PointCloudPostProcess.KeypointDetector, m_Config.PointCloudPostProcess.FeatureDetector, m_Config);
    }

    // Outlier removal
    void PointCloudPostProcessor::RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output) {
        m_OutlierRemover.setInputCloud(input);
        m_OutlierRemover.filter(*output);
    }

    // ICP alignment
    bool PointCloudPostProcessor::AlignPointCloud(PointCloudConstPtr source, PointCloudConstPtr target, PointCloudPtr result)
    {
        // get keypoints and perform ICP on keypoints
        KeypointDetectionResult sourceKeypoints;
        KeypointDetectionResult targetKeypoints;
        m_FeatureExtractor->ComputeKeypoints(source, nullptr, sourceKeypoints);
        m_FeatureExtractor->ComputeKeypoints(target, nullptr, targetKeypoints);

        PointCloudPtr alignmentResult { new pcl::PointCloud<PointType>() };

        // ICP on keypoints
        m_ICP.setInputCloud(source);
        m_ICP.setInputTarget(target);
        m_ICP.align(*alignmentResult);

        // apply the transformation to the complete point cloud
        Eigen::Matrix4f T = m_ICP.getFinalTransformation();

        /*
        // extract features from both point clouds and get correspondences
        FeatureDetectionResult sourceFeatures;
        FeatureDetectionResult targetFeatures;
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

        ExtractFeatures(source, sourceFeatures);
        ExtractFeatures(target, targetFeatures);

        switch (m_Config.PointCloudPostProcess.FeatureDetector)
        {
            case FEATURE_DETECTOR_FPFH:
            {
                pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr;
                corr.setInputSource(sourceFeatures.FPFHFeatures);
                corr.setInputTarget(targetFeatures.FPFHFeatures);
                corr.determineCorrespondences(*correspondences);
                break;
            }
            case FEATURE_DETECTOR_SHOT_COLOR:
            {
                // TODO: Implement SHOT COLOR
                break;
            }
        }

        // reject invalid correspondences using RANSAC
        pcl::CorrespondencesPtr validCorrespondences(new pcl::Correspondences());
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;

        rejector.setInputSource(source);
        rejector.setInputTarget(target);
        rejector.setInlierThreshold(2.5);
        rejector.setMaximumIterations(10);
        rejector.setRefineModel(false);
        rejector.setInputCorrespondences(correspondences);
        rejector.getCorrespondences(*validCorrespondences);

        // compute estimated rigid body transform
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> transformationEstimator;
        transformationEstimator.estimateRigidTransformation(*source, *target, *validCorrespondences, T);
        */

        //std::cout << "\nEstimated transform: \n" << T << std::endl;

        // apply transform to source
        pcl::transformPointCloud(*source, *result, T);

        return true;
    }

    // Extract features from point cloud
    void PointCloudPostProcessor::ExtractFeatures(PointCloudConstPtr cloud, FeatureDetectionResult& result) const
    {
        // TODO: uncomment when using keypoint detector that uses normals
        NormalsPtr normals = NormalsPtr(new pcl::PointCloud<pcl::Normal>());
        //m_FeatureExtractor->ComputeNormals(cloud, normals);

        // compute keypoints
        KeypointDetectionResult keypointsResult;
        m_FeatureExtractor->ComputeKeypoints(cloud, normals, keypointsResult);

        // compute keypoint normals
        NormalsPtr keypointNormals = NormalsPtr(new pcl::PointCloud<pcl::Normal>());
        m_FeatureExtractor->ComputeNormals(keypointsResult.Keypoints, keypointNormals);

        // compute features using keypoints
        m_FeatureExtractor->ComputeFeatures(keypointsResult.Keypoints, keypointNormals, result);
    }

    // Setters
    void PointCloudPostProcessor::SetMinimumNeighboursOutlierRemoval(int k) {
        m_OutlierRemover.setMeanK(k);
    }

    void PointCloudPostProcessor::SetStdDevOutlierRemoval(double std) {
        m_OutlierRemover.setStddevMulThresh(std);
    }
}
