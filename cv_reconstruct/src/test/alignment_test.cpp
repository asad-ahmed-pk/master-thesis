//
// alignment_test.cpp
// Test alignment algorithms
//

#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>

void GetKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void GetSHOTFeatureDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::SHOT1344>::Ptr output);
void GetNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::Normal>::Ptr normals);
void AlignByFeatures(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result);
void AlignByICP(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result);

#define SHOT_RADIUS 50.0f
#define NORMAL_RADIUS 50.0f

int main(int argc, char** argv)
{
    // read in 2 point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target{new pcl::PointCloud<pcl::PointXYZRGB>()};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source{new pcl::PointCloud<pcl::PointXYZRGB>()};
    pcl::io::loadPCDFile("frame_0.pcd", *target);
    pcl::io::loadPCDFile("frame_1.pcd", *source);

    std::cout << "\nPoint clouds loaded from disk. Performing registration.";

    // time registration run
    auto start = std::chrono::high_resolution_clock::now();

    // align using feature based registration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result { new pcl::PointCloud<pcl::PointXYZRGB>() };
    AlignByFeatures(source, target, result);

    // calculate time taken
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\nTotal time: " << duration << "ms" << std::endl;

    // save to disk
    pcl::io::savePCDFileBinary("alignment_test_output.pcd", *result);
    std::cout << "\nSaved result of alignment to disk";

    std::cout << std::endl;
    return 0;
}

void GetNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree { new pcl::search::KdTree<pcl::PointXYZRGB>() };

    normalEstimation.setInputCloud(input);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(NORMAL_RADIUS);
    normalEstimation.compute(*normals);
}

void GetKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> keypoints;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree { new pcl::search::KdTree<pcl::PointXYZRGB>() };
    sift.setSearchMethod(tree);
    sift.setScales(2.0, 8, 6);
    sift.setMinimumContrast(0.0);

    sift.setInputCloud(input);
    sift.compute(keypoints);

    pcl::copyPointCloud(keypoints, *output);
}

void GetSHOTFeatureDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::SHOT1344>::Ptr output)
{
    pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree { new pcl::search::KdTree<pcl::PointXYZRGB>() };

    pcl::PointCloud<pcl::Normal>::Ptr normals { new pcl::PointCloud<pcl::Normal>() };
    GetNormals(input, normals);

    shot.setRadiusSearch(SHOT_RADIUS);
    shot.setInputCloud(input);
    shot.setInputNormals(normals);
    shot.setSearchMethod(tree);

    shot.compute(*output);
}

void AlignByFeatures(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result)
{
    // keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeypoints { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypoints { new pcl::PointCloud<pcl::PointXYZRGB>() };
    GetKeypoints(source, sourceKeypoints); GetKeypoints(target, targetKeypoints);
    std::cout << "\nKeypoints computed";

    // SHOT descriptors
    pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors { new pcl::PointCloud<pcl::SHOT1344>() };
    pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors { new pcl::PointCloud<pcl::SHOT1344>() };

    GetSHOTFeatureDescriptors(sourceKeypoints, sourceDescriptors);
    GetSHOTFeatureDescriptors(targetKeypoints, targetDescriptors);

    std::cout << "\nSHOT Feature descriptors for keypoints computed";

    // correspondences
    boost::shared_ptr<pcl::Correspondences> correspondences { new pcl::Correspondences() };
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> correspondenceEstimation;
    correspondenceEstimation.setInputCloud(source);
    correspondenceEstimation.setInputTarget(target);
    correspondenceEstimation.determineCorrespondences(*correspondences);

    size_t correspondencesBefore = correspondences->size();
    std::cout << "\nDetermined " << correspondencesBefore << " correspondences";

    // rejection by RANSAC
    boost::shared_ptr<pcl::Correspondences> validCorrespondences { new pcl::Correspondences() };
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> correspondenceRejectorRANSAC;

    correspondenceRejectorRANSAC.setInputSource(source);
    correspondenceRejectorRANSAC.setInputTarget(target);
    correspondenceRejectorRANSAC.setInlierThreshold(3.0);
    correspondenceRejectorRANSAC.setMaximumIterations(100);
    correspondenceRejectorRANSAC.setInputCorrespondences(correspondences);
    correspondenceRejectorRANSAC.getCorrespondences(*validCorrespondences);

    size_t correspondencesAfter = validCorrespondences->size();
    std::cout << "\nRANSAC Rejected " << correspondencesBefore - correspondencesAfter << " correspondences";
    //Eigen::Matrix4f T = correspondenceRejectorRANSAC.getBestTransformation();

    // SVD transformation estimation
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> transformationEstimation;
    Eigen::Matrix4f T;

    transformationEstimation.estimateRigidTransformation(*source, *target, *validCorrespondences, T);
    std::cout << "\nEstimated Transform: \n" << T << std::endl;

    pcl::transformPointCloud(*source, *result, T);
    *result += *target;
}

void AlignByICP(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr result)
{
    // keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeypoints { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypoints { new pcl::PointCloud<pcl::PointXYZRGB>() };
    GetKeypoints(source, sourceKeypoints); GetKeypoints(target, targetKeypoints);

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    icp.setMaxCorrespondenceDistance (1000);
    icp.setMaximumIterations (1000);
    icp.setRANSACIterations(1000);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);

    icp.setInputTarget(target);
    icp.setInputCloud(source);
    icp.align(*result);

    Eigen::Matrix4f T = icp.getFinalTransformation();
    std::cout << "\nTransform: \n" << T << std::endl;

    *result += *target;
}
