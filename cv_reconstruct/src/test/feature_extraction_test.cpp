//
// feature_extraction_test.cpp
// Test feature extraction, and point cloud correspondence computation
//

#include <thread>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "tests_common.hpp"
#include "point_cloud/point_cloud_constants.hpp"

#include "point_cloud/FeatureDetectionResult.hpp"
#include "point_cloud/FeatureExtractor.hpp"

int main(int argc, char** argv)
{
    // read in 2 clouds
    PointCloud::PointCloudPtr cloud1 { new pcl::PointCloud<PointCloud::PointType>() };
    pcl::io::loadPCDFile("cloud_0.pcd", *cloud1);

    PointCloud::PointCloudPtr cloud2 { new pcl::PointCloud<PointCloud::PointType>() };
    pcl::io::loadPCDFile("cloud_1.pcd", *cloud2);

    std::cout << "\nLoaded point cloud files";

    // get config
    Config::ConfigParser parser;
    Config::Config config = std::move(parser.ParseConfig());

    // feature extractor will determine keypoints, compute normals, and generate descriptors
    PointCloud::FeatureExtractor featureExtractor(PointCloud::KEYPOINT_SIFT, PointCloud::FEATURE_DETECTOR_FPFH, config);

    // compute keypoints
    PointCloud::PointCloudPtr keypoints1 { new pcl::PointCloud<PointCloud::PointType>() };
    PointCloud::PointCloudPtr keypoints2 { new pcl::PointCloud<PointCloud::PointType>() };
    PointCloud::KeypointDetectionResult keypoints1Result;
    PointCloud::KeypointDetectionResult keypoints2Result;
    featureExtractor.ComputeKeypoints(cloud1, nullptr, keypoints1Result);
    featureExtractor.ComputeKeypoints(cloud2, nullptr, keypoints2Result);

    // copy SIFT result to point cloud
    pcl::copyPointCloud(*keypoints1Result.SIFTKeypoints, *keypoints1);
    pcl::copyPointCloud(*keypoints2Result.SIFTKeypoints, *keypoints2);

    // save keypoints to disk with colour
    for (int i = 0; i < keypoints1->points.size(); i++) {
        keypoints1->points[i].b = 255;
    }
    for (int i = 0; i < keypoints2->points.size(); i++) {
        keypoints2->points[i].r = 255;
    }

    PointCloud::PointCloudPtr combinedKeypoints { new pcl::PointCloud<pcl::PointXYZRGB>() };
    *combinedKeypoints += *keypoints1;
    *combinedKeypoints += *keypoints2;
    pcl::io::savePCDFileBinary("keypoints.pcd", *combinedKeypoints);

    std::cout << "\nComputed SIFT keypoints";

    // compute the normals for these keypoints
    PointCloud::NormalsPtr normals1 { new pcl::PointCloud<pcl::Normal>() };
    PointCloud::NormalsPtr normals2 { new pcl::PointCloud<pcl::Normal>() };
    featureExtractor.ComputeNormals(keypoints1, normals1);
    featureExtractor.ComputeNormals(keypoints2, normals2);

    std::cout << "\nComputed keypoint normals";

    // extract features
    PointCloud::FeatureDetectionResult cloud1Result;
    PointCloud::FeatureDetectionResult cloud2Result;
    featureExtractor.ComputeFeatures(keypoints1, normals1, cloud1Result);
    featureExtractor.ComputeFeatures(keypoints2, normals2, cloud2Result);

    std::cout << "\nComputed feature descriptors";

    // correspondences
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr;
    corr.setInputSource(cloud2Result.FPFHFeatures);
    corr.setInputTarget(cloud1Result.FPFHFeatures);
    corr.determineCorrespondences(*correspondences);

    std::cout << "\nComputed correspondences";

    // reject invalid correspondences using RANSAC
    pcl::CorrespondencesPtr validCorrespondences(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointCloud::PointType> rejector;

    rejector.setInputSource(keypoints2);
    rejector.setInputTarget(keypoints1);
    rejector.setInlierThreshold(2.5);
    rejector.setMaximumIterations(100);
    rejector.setRefineModel(false);
    rejector.setInputCorrespondences(correspondences);
    rejector.getCorrespondences(*validCorrespondences);

    std::cout << "\nCorrespondences refined using RANSAC";
    std::cout.flush();

    // remove and show only 10 correspondences
    int i = 0;
    pcl::CorrespondencesPtr filteredCorrespondences(new pcl::Correspondences());
    for (auto iter = validCorrespondences->begin(); iter != validCorrespondences->end(); iter++) {
        filteredCorrespondences->push_back(*iter);
        i++;
        if (i == 20) break;
    }

    // combined cloud for visualization purposes
    PointCloud::PointCloudPtr combinedCloud { new pcl::PointCloud<PointCloud::PointType>() };
    *combinedCloud += *cloud1;
    *combinedCloud += *cloud2;

    // visualize the correspondences
    auto viewer = rgbVis(combinedCloud);
    viewer->addCorrespondences<PointCloud::PointType>(keypoints2, keypoints1, *filteredCorrespondences);

    // visualise using PCL
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    return 0;
}

