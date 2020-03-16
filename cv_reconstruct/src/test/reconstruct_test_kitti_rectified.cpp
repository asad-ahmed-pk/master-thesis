//
// reconstruct_test_kitti_rectified.cpp
// Test stereo reconstruction functions on KITTI rectified data
//

#include "camera/CameraCompute.hpp"
#include "camera/CameraCalib.hpp"
#include "camera/CameraCalibParser.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "config/ConfigParser.hpp"
#include "point_cloud/PointCloudPostProcessor.hpp"

#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Test file names (placed in same directory as executable)
#define LEFT_IMG_NAME "0l.png"
#define RIGHT_IMG_NAME "0r.png"
#define CALIB_FILE_NAME "stereo_calib.json"

int main(int argc, char** argv)
{
    // parse calib from file
    Camera::Calib::StereoCalib stereoCalib;
    Camera::CameraCalibParser parser;
    if (!parser.ParseStereoCalibJSONFile(CALIB_FILE_NAME, stereoCalib)) {
        std::cerr << "\nFailed to parse calib file" << std::endl;
        return -1;
    }

    // parse config file
    Config::ConfigParser configParser;
    Config::Config config = std::move(configParser.ParseConfig());

    // create camera compute module and get updated stereo setup with rectification applied
    Camera::CameraCompute cameraCompute { stereoCalib };

    // create 3d reconstructor and generate disparity map
    Reconstruct::Reconstruct3D reconstructor(stereoCalib, config);

    // read in test images
    cv::Mat leftImage = cv::imread(LEFT_IMG_NAME, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(RIGHT_IMG_NAME, cv::IMREAD_COLOR);

    // generate disparity maps
    cv::Mat disparity = reconstructor.GenerateDisparityMap(leftImage, rightImage);
    std::cout << "\nDisparity map computed successfully" << std::endl;
    cv::imwrite("disparity_map.png", disparity);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(std::move(reconstructor.Triangulate3D(disparity, leftImage, rightImage))));

    // remove outliers
    PointCloud::PointCloudPostProcessor pointCloudPostProcessor(config);
    pointCloudPostProcessor.SetMinimumNeighboursOutlierRemoval(config.PointCloudPostProcess.OutlierMinK);
    pointCloudPostProcessor.SetStdDevOutlierRemoval(config.PointCloudPostProcess.OutlierStdDevThreshold);
    pointCloudPostProcessor.RemoveOutliers(pointCloud, pointCloud);

    // save point cloud file
    std::cout << "\nSaving point cloud file..." << std::endl;
    pcl::io::savePCDFileBinary("generated_point_cloud.pcd", *pointCloud);

    std::cout << "Done";
    std::cout << std::endl;

    return 0;
}

