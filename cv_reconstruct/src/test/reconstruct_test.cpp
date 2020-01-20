//
// reconstruct_test.cpp
// Test stereo reconstruction functions
//

#include "camera/CameraCompute.hpp"
#include "camera/CameraSettings.hpp"
#include "reconstruct/Reconstruct3D.hpp"

#include <string>
#include <iostream>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// constants for camera calibration from dataset
const Eigen::Matrix3f K1 = (Eigen::Matrix3f() << 837.619011, 0.0, 522.434637,
                                                0.0, 839.808333, 402.367400,
                                                0.0, 0.0, 1.0).finished();

const Eigen::Matrix3f K2 = (Eigen::Matrix3f() << 837.619011, 0.0, 522.434637,
                                                 0.0, 839.808333, 402.367400,
                                                 0.0, 0.0, 1.0).finished();

const Eigen::Vector2i IMAGE_RESOLUTION {1024, 768 };

const Eigen::Matrix3f R = (Eigen::Matrix3f() << 9.9997625494747e-001, -6.3729476131001e-003, -2.6220373684323e-003,
                                                6.3750339453031e-003, 9.9997936870410e-001, 7.8810427338438e-004,
                                                2.6169607251553e-003, -8.0480113703670e-004, 9.9999625189882e-001).finished();

const Eigen::Vector3f T = (Eigen::Vector3f() << 1.194711e-001, -3.144088e-004, 1.423872e-004).finished();

// Test image paths
const std::string LEFT_IMG_PATH { "test_images/img_00_left.jpg" };
const std::string RIGHT_IMG_PATH { "test_images/img_00_right.jpg" };

int main(int argc, char** argv)
{
    // distortion co-effs
    Eigen::VectorXf D(8);
    D << 0, 0, 0, 0, 0, 0, 0, 0;

    // left camera settings
    Camera::Settings::CameraSettings leftCamSettings;
    leftCamSettings.ImageResolutionInPixels = IMAGE_RESOLUTION;
    leftCamSettings.D = D;
    leftCamSettings.K = K1;

    // right camera settings
    Camera::Settings::CameraSettings rightCamSettings;
    rightCamSettings.ImageResolutionInPixels = IMAGE_RESOLUTION;
    rightCamSettings.D = D;
    rightCamSettings.K = K2;

    // stereo settings
    Camera::Settings::StereoCameraSettings stereoSetup;
    stereoSetup.LeftCamSettings = leftCamSettings;
    stereoSetup.RightCamSettings = rightCamSettings;
    stereoSetup.R = R;
    stereoSetup.T = T;

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
    //std::cout << "\n" << disparity << std::endl;
    cv::imwrite("disparity_map.png", disparity);

    std::cout << std::endl;
    return 0;
}

