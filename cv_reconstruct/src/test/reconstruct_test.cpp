//
// reconstruct_test.cpp
// Test stereo reconstruction functions
//

#include "camera/CameraCompute.hpp"
#include "camera/CameraSettings.hpp"
#include "camera/CameraCalibParser.hpp"
#include "reconstruct/Reconstruct3D.hpp"

#include <string>
#include <iostream>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Test image paths
const std::string LEFT_IMG_PATH { "test_images/img_00_left.jpg" };
const std::string RIGHT_IMG_PATH { "test_images/img_00_right.jpg" };

int main(int argc, char** argv)
{
    // parse calib from file
    Camera::Settings::StereoCameraSettings stereoSetup;
    Camera::CameraCalibParser parser;
    parser.ParseStereoCalibJSONFile("../resources/calib/test_calib.json", stereoSetup);

    // create camera compute module and get updated stereo setup with rectification applied
    Camera::CameraCompute cameraCompute { stereoSetup };
    stereoSetup = std::move(cameraCompute.GetRectifiedStereoSettings());

    // create 3d reconstructor and generate disparity map
    Reconstruct::Reconstruct3D reconstructor { stereoSetup };

    // read in test images
    cv::Mat leftImage = cv::imread(LEFT_IMG_PATH, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(RIGHT_IMG_PATH, cv::IMREAD_COLOR);

    cv::Mat disparity = reconstructor.GenerateDisparityMap(leftImage, rightImage);

    std::cout << "\nDisparity map computed successfully" << std::endl;
    cv::imwrite("disparity_map.png", disparity);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud = reconstructor.GeneratePointCloud(disparity, leftImage);

    // save point cloud file
    std::cout << "\nSaving point cloud file..." << std::endl;
    pcl::io::savePCDFileBinary("generated_point_cloud.pcd", pointCloud);

    std::cout << "Done";
    std::cout << std::endl;
    return 0;
}

