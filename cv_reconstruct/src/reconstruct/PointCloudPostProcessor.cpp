//
// PointCloudPostProcessor.cpp
// Post processes generated point clouds (registration, outlier removal, etc...)
//

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "reconstruct/PointCloudPostProcessor.hpp"

typedef pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> FPFH;

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

        // setup feature descriptor
        m_FeatureDescriptor = FPFH::Ptr(new FPFH());
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
        // downsample point clouds
        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::UniformSampling<pcl::PointXYZRGB> uniformSampling;
        uniformSampling.setInputCloud(source);
        uniformSampling.setRadiusSearch(10);
        uniformSampling.filter(*sourceDownsampled);

        uniformSampling.setInputCloud(target);
        uniformSampling.filter(*targetDownsampled);
        */

        // compute keypoints
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypoints(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr siftKeypoint(new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
        siftKeypoint->setScales(2.0, 16, 8);
        siftKeypoint->setMinimumContrast(0.0);

        siftKeypoint->setInputCloud(source);
        siftKeypoint->compute(*sourceKeypoints);
        siftKeypoint->setInputCloud(target);
        siftKeypoint->compute(*targetKeypoints);

        // compute features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures(new pcl::PointCloud<pcl::FPFHSignature33>());
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures(new pcl::PointCloud<pcl::FPFHSignature33>());

        ComputeFPFHFeatures(sourceKeypoints, sourceFeatures);
        ComputeFPFHFeatures(targetKeypoints, targetFeatures);

        // compute correspondences
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corrEstimation;

        corrEstimation.setInputSource(sourceFeatures);
        corrEstimation.setInputTarget(targetFeatures);
        corrEstimation.determineCorrespondences(*correspondences);

        // reject invalid correspondences using RANSAC
        pcl::CorrespondencesPtr validCorrespondences(new pcl::Correspondences());
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;

        rejector.setInputSource(source);
        rejector.setInputTarget(target);
        rejector.setInlierThreshold(2.5);
        rejector.setMaximumIterations(20);
        rejector.setRefineModel(false);
        rejector.setInputCorrespondences(correspondences);
        rejector.getCorrespondences(*validCorrespondences);

        // compute estimated rigid body transform
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> transformationEstimator;
        transformationEstimator.estimateRigidTransformation(*source, *target, *validCorrespondences, T);

        //std::cout << "\nEstimated transform: \n" << T << std::endl;

        // apply transform to source
        pcl::transformPointCloud(*source, *result, T);

        // ICP as last step for final refinement
        /*
        m_ICP.setInputSource(source);
        m_ICP.setInputTarget(target);
        m_ICP.align(*result);

        return m_ICP.hasConverged();
         */

        return true;
    }

    // Compute FPFH features
    void PointCloudPostProcessor::ComputeFPFHFeatures(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) const
    {
        // compute normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        EstimateNormals(keypoints, normals);

        // compute features
        m_FeatureDescriptor->setInputNormals(normals);
        m_FeatureDescriptor->setInputCloud(keypoints);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        m_FeatureDescriptor->setSearchMethod(tree);
        m_FeatureDescriptor->setRadiusSearch(300);
        m_FeatureDescriptor->compute(*features);

        // compute persistent features at multiple scales
        /*
        std::vector<float> scales { 0.5f, 1.0f, 1.5f };
        pcl::MultiscaleFeaturePersistence<pcl::PointXYZRGB, pcl::FPFHSignature33> persistentFeatureCompute;

        persistentFeatureCompute.setScalesVector(scales);
        persistentFeatureCompute.setAlpha(1.3f);
        persistentFeatureCompute.setFeatureEstimator(m_FeatureDescriptor);
        persistentFeatureCompute.setDistanceMetric(pcl::CS);
        persistentFeatureCompute.determinePersistentFeatures(*features, keypoints);
         */
    }

    // Estimate normals
    void PointCloudPostProcessor::EstimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) const
    {
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

        normalEstimation.setSearchMethod(tree);
        normalEstimation.setRadiusSearch(250);
        normalEstimation.compute(*normals);
    }

    // Setters
    void PointCloudPostProcessor::SetMinimumNeighboursOutlierRemoval(int k) {
        m_OutlierRemover.setMeanK(k);
    }

    void PointCloudPostProcessor::SetStdDevOutlierRemoval(double std) {
        m_OutlierRemover.setStddevMulThresh(std);
    }
}
