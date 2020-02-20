//
// point_cloud_constants.hpp
// Contains enums and constants for point cloud processing
//

#ifndef MASTER_THESIS_POINT_CLOUD_CONSTANTS_HPP
#define MASTER_THESIS_POINT_CLOUD_CONSTANTS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace PointCloud
{
    // General point cloud pointer used throughout the pipeline
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudConstPtr;

    // Normals
    typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;

    // The point type this pipeline operates on
    typedef pcl::PointXYZRGB PointType;

    /// The supported keypoints used for point cloud processing
    enum KeypointType
    {
        KEYPOINT_SIFT,
        KEYPOINT_ISS_3D
    };

    /// The supported feature detectors for point cloud processing
    enum FeatureDetectorType
    {
        FEATURE_DETECTOR_FPFH,
        FEATURE_DETECTOR_SHOT_COLOR
    };
}

#endif //MASTER_THESIS_POINT_CLOUD_CONSTANTS_HPP
