//
// localization_test_kitti.cpp
// localization test on kitti dataset
//

#include <iostream>
#include <vector>
#include <thread>

#include <eigen3/Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "reconstruct/Reconstruct3D.hpp"
#include "reconstruct/Localizer.hpp"
#include "camera/CameraCalibParser.hpp"
#include "config/ConfigParser.hpp"
#include "pipeline/ReconstructionPipeline.hpp"

#include "tests_common.hpp"

// test file names
#define LOCALIZATION_DATA_FILE "localization_data.txt"

void AddXYZPattern(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void FillLocalizationData(std::vector<LocalizationData>& data);
Eigen::Matrix4f GetPose(const Pipeline::StereoFrame& frame);

int main(int argc, char** argv)
{
    // read in localization data and convert to frames
    std::vector<Pipeline::StereoFrame> frames;
    std::vector<LocalizationData> localizationData;
    std::vector<Eigen::Matrix4f> poses;

    // create test frame data
    //FillLocalizationData(localizationData);
    ReadLocalizationData(LOCALIZATION_DATA_FILE, localizationData);

    // not needed: maybe delete this
    ConvertLocalizationsToStereoFrames(localizationData, frames);

    // get calib and config
    Camera::Calib::StereoCalib stereoCalib;
    Config::Config config;
    GetCalibAndConfig(stereoCalib, config);

    // localizer module
    Reconstruct::Localizer localizer;

    // point clouds for each frame
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds;
    for (int i = 0; i < frames.size(); i++)
    {
        clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>());
        AddCameraDebugShape(clouds[i], (i == 0 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 ? 255 : 0));
        pcl::transformPointCloud(clouds[i], clouds[i], localizer.GetFrameWorldPose(frames[i]));
    }
    assert(clouds.size() == frames.size());

    // combine all into 1 scene
    pcl::PointCloud<pcl::PointXYZRGB> scene;
    for (int i = 0; i < clouds.size(); i++) {
        scene += clouds[i];
    }

    pcl::io::savePCDFileBinary("localization_test.pcd", scene);

    std::cout << std::endl;
    return 0;
}

// Test data
void FillLocalizationData(std::vector<LocalizationData>& data)
{
    // pose 1: no rotation, at origin
    LocalizationData data1;
    data1.roll = data1.pitch = data1.yaw = 0;
    data1.lat = data1.lon = data1.alt = 0;
    data.push_back(data1);

    // pose 2: forward 50m, look right
    LocalizationData data2;
    data2.roll = 0; data2.pitch = 0; data2.yaw = -M_PI_2;
    data2.lat = data2.lon = 0; data2.alt = 100;
    data.push_back(data2);
}

// Get pose
Eigen::Matrix4f GetPose(const Pipeline::StereoFrame& frame)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    pose.block(0, 0, 3, 3) = frame.Rotation;
    pose.block(0, 3, 3, 1) = frame.Translation;

    std::cout << "\nPose:\n" << pose << std::endl;

    return pose;
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
