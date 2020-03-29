//
// ReconstructionSystem.cpp
// Main system for 3D reconstruction. Computes keyframes, and creates 3D map.
//

#include "system/ReconstructionSystem.hpp"

#define RAD_TO_DEG (180.0/3.141592653589793238463)

namespace System
{
    // Constructor
    ReconstructionSystem::ReconstructionSystem(const Config::Config& config, const Camera::Calib::StereoCalib& stereoCalib) : m_Config(config)
    {
        // init all sub systems and components
        
        // 3D reconstruction module (shared by many subsystems)
        m_3DReconstructor = std::make_shared<Reconstruct::Reconstruct3D>(stereoCalib, config);
        
        // mapping subsystem: performs windowed BA and local optimisation of the map
        m_MappingSystem = std::make_shared<MappingSystem>();
        m_MappingSystem->StartOptimisationThread();
        
        // tracker: tracks frames for local mapping and quick localisation
        m_Tracker = std::make_unique<Tracker>(m_FeatureExtractor, m_3DReconstructor, m_MappingSystem);
    }

    // Process stereo frame
    void ReconstructionSystem::ProcessStereoFrame(const Pipeline::StereoFrame& stereoFrame)
    {
        // disparity image (used as basis for 3D reconstruction)
        cv::Mat disparity;
        cv::Mat leftImage; cv::Mat rightImage;

        // check if stereo rectification is needed (from config)
        if (m_Config.Reconstruction.ShouldRectifyImages) {
            m_3DReconstructor->RectifyImages(stereoFrame.LeftImage, stereoFrame.RightImage, leftImage, rightImage);
            disparity = m_3DReconstructor->GenerateDisparityMap(leftImage, rightImage);
        }
        else {
            leftImage = stereoFrame.LeftImage;
            rightImage = stereoFrame.RightImage;
        }

        // create disparity image from stereo frame
        disparity = m_3DReconstructor->GenerateDisparityMap(leftImage, rightImage);

        // create the tracking frame for this stereo frame and pass to tracker to track
        std::shared_ptr<TrackingFrame> frame { new TrackingFrame(leftImage, disparity, m_FeatureExtractor, m_3DReconstructor) };
        m_Tracker->TrackFrame(frame);
    }

    // Shutdown request
    void ReconstructionSystem::RequestShutdown() {
        m_RequestedShutdown = true;
    }

    // Get current map
    void ReconstructionSystem::GetCurrentBuiltMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
        m_MappingSystem->GetMap(cloud);
    }
}
