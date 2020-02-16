//
// localization_test_kitti.cpp
// localization test on kitti dataset
//

#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>

#include <eigen3/Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/highgui/highgui.hpp>

#include "reconstruct/Reconstruct3D.hpp"
#include "camera/CameraCalibParser.hpp"
#include "config/ConfigParser.hpp"

// test file names
#define LOCALIZATION_DATA_FILE "localization_data.txt"
#define CALIB_FILE "stereo_calib.json"
#define LEFT_IMG_NAME "img_left.png"
#define RIGHT_IMG_NAME "img_right.png"

struct LocalizationData {
    float lat;
    float lon;
    float alt;
    float roll;
    float pitch;
    float yaw;
};

Eigen::Matrix3f RotationMatrixFromEuler(float pitch, float yaw, float roll);
Reconstruct::StereoFrame ConvertToFrame(const LocalizationData& data);
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
void AddXYZPattern(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void PointClicked(const pcl::visualization::PointPickingEvent& event);

int main(int argc, char** argv)
{
    // read in GPS data
    std::ifstream fs(LOCALIZATION_DATA_FILE, std::ios::in);
    std::string line;

    LocalizationData data{};

    std::vector<Reconstruct::StereoFrame> frames;
    std::vector<LocalizationData> localizationData;

    // read in localization data from test flat file
    while (std::getline(fs, line))
    {
        // get lat,lon,alt from each line
        std::istringstream is(line);
        is >> data.lat >> data.lon >> data.alt >> data.roll >> data.pitch >> data.yaw;
        localizationData.push_back(data);
    }

    // convert to vector of frames
    int i = 0;
    std::transform(localizationData.begin(), localizationData.end(), std::back_inserter(frames), ConvertToFrame);
    for (Reconstruct::StereoFrame& frame : frames)
    {
        frame.ID = i++;
        frame.LeftImage = cv::imread(std::to_string(frame.ID) + "l.png", cv::IMREAD_COLOR);
        frame.RightImage = cv::imread(std::to_string(frame.ID) + "r.png", cv::IMREAD_COLOR);
    }

    // parse calib from file
    Camera::Calib::StereoCalib stereoCalib;
    Camera::CameraCalibParser parser;
    if (!parser.ParseStereoCalibJSONFile(CALIB_FILE, stereoCalib)) {
        std::cerr << "\nFailed to parse calib file" << std::endl;
        return -1;
    }

    // parse config file
    Config::ConfigParser configParser;
    Config::Config config = configParser.ParseConfig();

    // prepare reconstruction module
    Reconstruct::Reconstruct3D reconstructor(stereoCalib, false);
    reconstructor.SetBlockMatcherType(Reconstruct::STEREO_BLOCK_MATCHER);
    reconstructor.SetStereoBMWindowSize(config.WindowSize);
    reconstructor.SetStereoBMNumDisparities(config.NumDisparities);

    // prepare localization module
    Reconstruct::Localizer localizer;

    // main process loop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input { new pcl::PointCloud<pcl::PointXYZRGB>() };

    //AddXYZPattern(input);

    if (frames.empty()) {
        std::cerr << "\nError: Frames list is empty. Aborting" << std::endl;
        return 1;
    }

    for (const auto& frame : frames)
    {
        reconstructor.ProcessFrame(frame, *temp);
        *pointCloud += *temp;
        temp->clear();

        /*
        input->clear();
        pcl::PointXYZRGB p{};
        p.z = 10;
        if (i == 0) p.r = 255;
        if (i == 1) p.g = 255;
        if (i == 2) p.b = 255;
        input->push_back(p);

        localizer.TransformPointCloud(frame, *input, *temp);
        *pointCloud += *temp;
        temp->clear();

        i++;
         */
    }

    // visualise using PCL
    pcl::visualization::PCLVisualizer::Ptr viewer = rgbVis(pointCloud);
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // save to disk
    std::cout << "\nSaving PCD to disk" << std::endl;
    pcl::io::savePCDFileBinary("localized.pcd", *pointCloud);

    std::cout << std::endl;
    return 0;
}

// Convert to stereo frame
Reconstruct::StereoFrame ConvertToFrame(const LocalizationData& data)
{
    Reconstruct::StereoFrame frame{};
    frame.Translation = Eigen::Vector3f(data.lat, data.lon, data.alt);
    frame.Rotation = RotationMatrixFromEuler(data.pitch, data.yaw, data.roll);
    return std::move(frame);
}

// Get 3x3 rotation matrix from euler angles (Z - Forward, X - Right, Y - Up)
Eigen::Matrix3f RotationMatrixFromEuler(float pitch, float yaw, float roll)
{
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotation = q.matrix();

    return rotation;
}

// Visualise point cloud (source: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php)
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->registerPointPickingCallback(PointClicked);

    return (viewer);
}

// Add points to point cloud in X,Y,Z pattern
void AddXYZPattern(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    const float scale = 1.0;

    pcl::PointXYZRGB x{};
    x.x = 1.0f * scale;
    x.r = 255;

    pcl::PointXYZRGB y{};
    y.y = 1.0f * scale;
    y.g = 255;

    pcl::PointXYZRGB z{};
    z.z = 1.0f * scale;
    z.b = 255;

    cloud->push_back(x);
    cloud->push_back(y);
    cloud->push_back(z);
}

// Mouse click event
void PointClicked(const pcl::visualization::PointPickingEvent& event)
{
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "\nPoint #" << event.getPointIndex();
    std::cout << "\nClicked point at: (" << x << ", " << y << ", " << z << ")";
}