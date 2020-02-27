//
// pipeline_test.cpp
// Test full processing of the pipeline from input to output on a subset of frames
//

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>

#include "pipeline/StereoFrame.hpp"
#include "pipeline/ReconstructionPipeline.hpp"
#include "tests_common.hpp"

#define LOCALIZATION_DATA_FILE "pipeline_localization_data.txt"

int main(int argc, char** argv)
{
    // read in localization data and convert to frames
    std::vector<Pipeline::StereoFrame> frames;
    std::vector<LocalizationData> localizationData;

    ReadLocalizationData(LOCALIZATION_DATA_FILE, localizationData);
    ConvertLocalizationsToStereoFrames(localizationData, frames);

    // get calib and config
    Camera::Calib::StereoCalib stereoCalib;
    Config::Config config;
    GetCalibAndConfig(stereoCalib, config);

    // prepare pipeline processor
    Pipeline::ReconstructionPipeline pipeline(config, stereoCalib, true);

    // main process loop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp { new pcl::PointCloud<pcl::PointXYZRGB>() };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input { new pcl::PointCloud<pcl::PointXYZRGB>() };

    if (frames.empty()) {
        std::cerr << "\nError: Frames list is empty. Aborting" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI> greyscaleCloud;
    for (const auto& frame : frames)
    {
        std::cout << "\nProcessing frame #" << frame.ID << std::endl;
        pipeline.ProcessFrame(frame, temp);

        // add colour
        /*
        for (int i = 0; i < temp->points.size(); i++)
        {
            temp->points[i].rgb = 0;
            if (frame.ID == 0) {
                temp->points[i].r = 255;
            }
            else {
                temp->points[i].b = 255;
            }
        }

        pcl::io::savePCDFileBinary("cloud_" + std::to_string(frame.ID) + ".pcd", *temp);
         */

        // save greyscale cloud
        ConvertToGreyScale(*temp, greyscaleCloud);
        pcl::io::savePCDFileASCII("cloud_" + std::to_string(frame.ID) + ".ascii", greyscaleCloud);
        pcl::io::savePCDFileBinary("cloud_" + std::to_string(frame.ID) + ".pcd", *temp);
        greyscaleCloud.clear();
        *pointCloud += *temp;
        temp->clear();
    }

    // visualise using PCL
    pcl::visualization::PCLVisualizer::Ptr viewer = rgbVis(pointCloud);
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // save to disk
    std::cout << "\nSaving PCD to disk" << std::endl;
    pcl::io::savePCDFileBinary("pipeline_output.pcd", *pointCloud);

    std::cout << std::endl;
    return 0;
}
