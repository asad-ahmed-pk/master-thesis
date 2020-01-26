//
// reconstruct_test.cpp
// Test stereo reconstruction functions
//

#include "camera/CameraCompute.hpp"
#include "camera/CameraCalib.hpp"
#include "camera/CameraCalibParser.hpp"
#include "reconstruct/Reconstruct3D.hpp"

#include <string>
#include <iostream>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/core/base.hpp>
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
    Camera::Calib::StereoCalib stereoSetup;
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
    cv::Mat disparity8;
    cv::normalize(disparity, disparity8, 0, 255, cv::NORM_MINMAX, CV_8U);

    std::cout << "\nDisparity map computed successfully" << std::endl;
    cv::imwrite("disparity_map.png", disparity);
    cv::imwrite("disparity_map_8.png", disparity8);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    //pointCloud = reconstructor.GeneratePointCloud(disparity, leftImage);

    cv::Mat leftRect, rightRect;
    leftRect = cv::imread("../resources/test_images/rect_img_00_left.jpg");
    rightRect = cv::imread("../resources/test_images/rect_img_00_right.jpg");
    pointCloud = reconstructor.Triangulate3D(disparity, leftRect, rightRect);

    // save point cloud file
    std::cout << "\nSaving point cloud file..." << std::endl;
    pcl::io::savePCDFileBinary("generated_point_cloud.pcd", pointCloud);

    std::cout << "Done";
    std::cout << std::endl;
    return 0;
}

