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

// Create a point with RGB and XYZ
pcl::PointXYZRGB CreatePoint(float x, float y, float z, int r, int g, int b);

// Convert point to text coords
std::string PointCoordsToString(const pcl::PointXYZRGB& point);

// Create rotation matrix from euler angles
Eigen::Matrix3f RotationMatrixFromEuler(float pitch, float yaw, float roll);

// Create a stereo frame from localization data
Pipeline::StereoFrame ConvertToFrame(const LocalizationData& data);

// Get a PCL visualiser for a RGB cloud
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

// PCL point clicked event handler
void PointClicked(const pcl::visualization::PointPickingEvent& event);

// Convert point cloud to greyscale
void ConvertToGreyScale(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointXYZI>& result);

// Mercator projection
Eigen::Vector3f ProjectGPSToMercator(float latitude, float longitude, float altitude);

// Get world pose from GPS in euclidean space for frame
Eigen::Matrix4f GetFrameWorldPose(const Pipeline::StereoFrame& frame);

// Camera debug shape
void AddCameraDebugShape(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int r, int g, int b, float scale = 10.0f);

#endif //MASTER_THESIS_TESTS_COMMON_HPP
