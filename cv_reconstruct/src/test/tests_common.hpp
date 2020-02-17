//
// tests_common.hpp
// Common functions used by all test programs
//

#ifndef MASTER_THESIS_TESTS_COMMON_HPP
#define MASTER_THESIS_TESTS_COMMON_HPP

#include <vector>
#include <string>

#include <eigen3/Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pipeline/StereoFrame.hpp"
#include "config/ConfigParser.hpp"
#include "camera/CameraCalibParser.hpp"

#define CALIB_FILE "stereo_calib.json"

// Localization data for testing
struct LocalizationData {
    float lat;
    float lon;
    float alt;
    float roll;
    float pitch;
    float yaw;
};

// Read data from localization data file
void ReadLocalizationData(const std::string& filename, std::vector<LocalizationData>& data);

// Convert all localization data to stereo frames
void ConvertLocalizationsToStereoFrames(const std::vector<LocalizationData>& data, std::vector<Pipeline::StereoFrame>& frames);

// Get calib and config file
bool GetCalibAndConfig(Camera::Calib::StereoCalib& calib, Config::Config& config);

// Create rotation matrix from euler angles
Eigen::Matrix3f RotationMatrixFromEuler(float pitch, float yaw, float roll);

// Create a stereo frame from localization data
Pipeline::StereoFrame ConvertToFrame(const LocalizationData& data);

// Get a PCL visualiser for a RGB cloud
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

// PCL point clicked event handler
void PointClicked(const pcl::visualization::PointPickingEvent& event);

#endif //MASTER_THESIS_TESTS_COMMON_HPP
