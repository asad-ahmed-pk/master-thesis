//
// ReconstructionSystem.hpp
// Main system for 3D reconstruction. Computes keyframes, and creates 3D map.
//

#ifndef MASTER_THESIS_RECONSTRUCTIONSYSTEM_HPP
#define MASTER_THESIS_RECONSTRUCTIONSYSTEM_HPP

#include <atomic>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "config/Config.hpp"
#include "reconstruct/Reconstruct3D.hpp"
#include "system/MappingSystem.hpp"
#include "point_cloud/PointCloudRegistration.hpp"

namespace Pipeline {
    class StereoFrame;
}

namespace Camera {
    namespace Calib {
        class StereoCalib;
    }
}

namespace System
{
    class Keyframe;

    class ReconstructionSystem
    {
    public:
        /// Construct an instance of the system using the system-wide config. Maintains its own 3D map.
        /// \param config The config with params for reconstruction
        /// \param stereoCalib The stereo calibration
        ReconstructionSystem(const Config::Config& config, const Camera::Calib::StereoCalib& stereoCalib);

        /// Destructor
        ~ReconstructionSystem() = default;

        /// Request system to shut down all processing and threads it is running
        void RequestShutdown();

        /// Process the stereo frame
        /// \param stereoFrame The stereo frame containing the left and right stereo images
        void ProcessStereoFrame(const Pipeline::StereoFrame& stereoFrame);

    private:
        void PruneDisparityImage(cv::Mat& disparity) const;

    private:
        Config::Config m_Config;
        std::atomic_bool m_RequestedShutdown { false };

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_PreviousPointCloud { nullptr };
        Eigen::Matrix4f m_PreviousEstimatedPose = Eigen::Matrix4f::Identity();

        PointCloud::PointCloudRegistration m_PointCloudRegistration;
        Reconstruct::Reconstruct3D m_3DReconstructor;
        MappingSystem m_MappingSystem;
    };
}

#endif //MASTER_THESIS_RECONSTRUCTIONSYSTEM_HPP
