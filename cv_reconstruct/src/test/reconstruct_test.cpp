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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Test file names (placed in same directory as executable)
#define LEFT_IMG_NAME "img_left.png"
#define RIGHT_IMG_NAME "img_right.png"
#define LEFT_IMG_RECTIFIED_NAME "img_left_rectified.png"
#define RIGHT_IMG_RECTIFIED_NAME "img_right_rectified.png"
#define CALIB_FILE_NAME "stereo_calib.json"

int main(int argc, char** argv)
{
    // parse calib from file
    Camera::Calib::StereoCalib stereoSetup;
    Camera::CameraCalibParser parser;
    if (!parser.ParseStereoCalibJSONFile(CALIB_FILE_NAME, stereoSetup)) {
        std::cerr << "\nFailed to parse calib file" << std::endl;
        return -1;
    }

    // create camera compute module and get updated stereo setup with rectification applied
    Camera::CameraCompute cameraCompute { stereoSetup };
    stereoSetup = std::move(cameraCompute.GetRectifiedStereoSettings());

    // create 3d reconstructor and generate disparity map
    Reconstruct::Reconstruct3D reconstructor { stereoSetup };

    // read in test images
    cv::Mat leftImage = cv::imread(LEFT_IMG_NAME, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(RIGHT_IMG_NAME, cv::IMREAD_COLOR);

    // rectify images
    cv::Mat leftImageRectified{};
    cv::Mat rightImageRectified{};
    reconstructor.RectifyImages(leftImage, rightImage, leftImageRectified, rightImageRectified);

    cv::imwrite(LEFT_IMG_RECTIFIED_NAME, leftImageRectified);
    cv::imwrite(RIGHT_IMG_RECTIFIED_NAME, rightImageRectified);

    // generate disparity maps
    cv::Mat disparity = reconstructor.GenerateDisparityMap(leftImageRectified, rightImageRectified);
    std::cout << "\nDisparity map computed successfully" << std::endl;
    cv::imwrite("disparity_map.png", disparity);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    //pointCloud = reconstructor.GeneratePointCloud(disparity, leftImage);
    pointCloud = reconstructor.Triangulate3D(disparity, leftImageRectified, rightImageRectified);

    // save point cloud file
    std::cout << "\nSaving point cloud file..." << std::endl;
    pcl::io::savePCDFileBinary("generated_point_cloud.pcd", pointCloud);

    std::cout << "Done";
    std::cout << std::endl;

    return 0;
}

