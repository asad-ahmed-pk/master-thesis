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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Test file names (placed in same directory as executable)
#define LEFT_IMG_NAME "0l.png"
#define RIGHT_IMG_NAME "0r.png"
#define CALIB_FILE_NAME "stereo_calib.json"

#define IMAGE_SCALE_FACTOR 0.5

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
    Config::Config config = configParser.ParseConfig();
    
    // apply scaling factor to intrinsics
    stereoCalib.LeftCameraCalib.K(0, 2) = stereoCalib.LeftCameraCalib.K(0, 2) * IMAGE_SCALE_FACTOR;
    stereoCalib.LeftCameraCalib.K(1, 2) = stereoCalib.LeftCameraCalib.K(0, 2) * IMAGE_SCALE_FACTOR;
    stereoCalib.LeftCameraCalib.K(0, 0) = stereoCalib.LeftCameraCalib.K(0, 0) * IMAGE_SCALE_FACTOR;
    stereoCalib.LeftCameraCalib.K(1, 1) = stereoCalib.LeftCameraCalib.K(1, 1) * IMAGE_SCALE_FACTOR;

    // create camera compute module and get updated stereo setup with rectification applied
    Camera::CameraCompute cameraCompute { stereoCalib };

    // create 3d reconstructor and generate disparity map
    Reconstruct::Reconstruct3D reconstructor(stereoCalib, config);

    // read in test images
    cv::Mat leftImage = cv::imread(LEFT_IMG_NAME, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(RIGHT_IMG_NAME, cv::IMREAD_COLOR);
    
    // downsize
    cv::resize(leftImage, leftImage, cv::Size(leftImage.cols * IMAGE_SCALE_FACTOR, leftImage.rows * IMAGE_SCALE_FACTOR));
    cv::resize(rightImage, rightImage, cv::Size(rightImage.cols * IMAGE_SCALE_FACTOR, rightImage.rows * IMAGE_SCALE_FACTOR));
    cv::imwrite("left_image_downsized.png", leftImage);
    cv::imwrite("right_image_downsized.png", rightImage);
    
    // generate disparity maps
    cv::Mat disparity = reconstructor.GenerateDisparityMap(leftImage, rightImage);
    std::cout << "\nDisparity map computed successfully" << std::endl;
    cv::imwrite("disparity_map.png", disparity);
    
    // true disparity
    disparity.convertTo(disparity, CV_32F, 1.0 / 16.0, 0.0);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(std::move(reconstructor.Triangulate3D(disparity, leftImage))));

    // remove outliers
    /*
    PointCloud::PointCloudPostProcessor pointCloudPostProcessor(config);
    pointCloudPostProcessor.SetMinimumNeighboursOutlierRemoval(config.PointCloudPostProcess.OutlierMinK);
    pointCloudPostProcessor.SetStdDevOutlierRemoval(config.PointCloudPostProcess.OutlierStdDevThreshold);
    pointCloudPostProcessor.RemoveOutliers(pointCloud, pointCloud);
    */

    // save point cloud file
    std::cout << "\nSaving point cloud file..." << std::endl;
    pcl::io::savePCDFileBinary("generated_point_cloud.pcd", *pointCloud);

    std::cout << "Done";
    std::cout << std::endl;

    return 0;
}

