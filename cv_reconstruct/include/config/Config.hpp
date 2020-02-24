//
// Config.hpp
// Stores the config for the reconstruction server
//

#ifndef MASTER_THESIS_CONFIG_HPP
#define MASTER_THESIS_CONFIG_HPP

#include <string>

#include "point_cloud/point_cloud_constants.hpp"
#include "reconstruct/Reconstruct3DTypes.hpp"

namespace Config
{
    struct Config
    {
        // Server
        struct Server
        {
            int ServerPort;

        } Server;

        // 3D Reconstruction
        struct Reconstruction
        {
            bool ShouldRectifyImages { true };
            Reconstruct::StereoBlockMatcherType BlockMatcherType { Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER };

            struct SBM {
                int NumDisparities { 16 };
                int WindowSize { 21 };
            } SBM;

            struct SGBM {
                int NumDisparities { 16 };
                int BlockSize { 128 };
                int UniquenessRatio { 2 };
                int SpeckleRange { 2 };
                int SpeckleWindowSize { 100 };
                int PreFilterCap { 10 };
                int MinDisparity { 0 };
            } SGBM;

        } Reconstruction;

        // Point cloud post processing
        struct PointCloudPostProcess
        {
            double OutlierStdDevThreshold { 1.0 };
            int OutlierMinK { 50 };
            PointCloud::KeypointType KeypointDetector;
            PointCloud::FeatureDetectorType FeatureDetector;

        } PointCloudPostProcess;

        // Point cloud registration
        struct PointCloudRegistration
        {
            // Params for ICP algorithm
            struct ICP {
                int NumMaxIterations { 20 };
                int NumRansacIterations { 20 };
                double TransformEpsilon { 1e-8 };
                double EuclideanFitnessEpsilon { 1.0 };
            } ICP;

        } PointCloudRegistration;

        // Point cloud feature detection
        struct PointCloudFeatureDetection
        {
            struct Normals {
                float Radius { 0.0 };
            } Normals;
            struct FPFH {
                float MinRadius { 0.0 };
            } FPFH;
            struct SHOTColor {

            } SHOTColor;

        } PointCloudFeatureDetection;

        // Point cloud keypoint detection
        struct PointCloudKeypointDetection
        {
            struct SIFT {
                float MinScale { 1.0 };
                int NumOctaves { 0 };
                int NumScalesPerOctave { 0 };
                float MinContrast { 0.0 };
            } SIFT;

        } PointCloudKeypointDetection;
    };
}

#endif //MASTER_THESIS_CONFIG_HPP
