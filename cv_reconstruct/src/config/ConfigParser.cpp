//
// ConfigParser.hpp
// Parses JSON config for the server settings
//

#include "config/ConfigParser.hpp"
#include "nlohmann/json.hpp"

#include <iostream>
#include <boost/filesystem.hpp>

namespace Config
{
#define CONFIG_DEFAULT_PATH "../../../cv_reconstruct/resources/config/config_default.json"
#define CONFIG_FILE_PATH "config.json"

    // Parse config
    Config ConfigParser::ParseConfig()
    {
        // check if config exists
        if (!boost::filesystem::exists(CONFIG_FILE_PATH)) {
            boost::filesystem::copy_file(CONFIG_DEFAULT_PATH, CONFIG_FILE_PATH);
        }

        std::ifstream fs(CONFIG_FILE_PATH, std::ios::in);
        nlohmann::json json;
        fs >> json;
        fs.close();

        // extract json into config record
        Config config;
        nlohmann::json serverConfig = json["config"]["server"];

        // server config
        config.Server.ServerPort = serverConfig["port"];

        // reconstruction config
        nlohmann::json reconstructionConfig = json["config"]["reconstruction"];
        config.Reconstruction.ShouldRectifyImages = reconstructionConfig["requires_rectification"];

        // block matcher parsed into enum
        std::string bmTypeString = reconstructionConfig["block_matcher"];
        if (bmTypeString == "stereo_bm") {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER;
        }
        else if (bmTypeString == "stereo_sgbm") {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_SEMI_GLOBAL_BLOCK_MATCHER;
        }
        else {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER;
        }

        config.Reconstruction.WindowSize = reconstructionConfig["window_size"];
        config.Reconstruction.NumDisparities = reconstructionConfig["num_disparities"];

        // point cloud post processing config
        nlohmann::json pointCloudPostProcessConfig = json["config"]["point_cloud_post_processing"];
        config.PointCloudPostProcess.OutlierMinK = pointCloudPostProcessConfig["outlier_min_k"];
        config.PointCloudPostProcess.OutlierStdDevThreshold = pointCloudPostProcessConfig["outlier_std_threshold"];

        // point cloud registration config
        nlohmann::json registrationConfig = json["config"]["point_cloud_registration"];
        config.PointCloudRegistration.ICP.EuclideanFitnessEpsilon = registrationConfig["ICP"]["euclidean_fitness_epsilon"];
        config.PointCloudRegistration.ICP.NumMaxIterations = registrationConfig["ICP"]["max_iterations"];
        config.PointCloudRegistration.ICP.NumRansacIterations = registrationConfig["ICP"]["ransac_iterations"];
        config.PointCloudRegistration.ICP.TransformEpsilon = registrationConfig["ICP"]["transformation_epsilon"];

        // keypoint detector parsed into enum
        const std::string keypointDetector = pointCloudPostProcessConfig["keypoint_detector"];
        config.PointCloudPostProcess.KeypointDetector = PointCloud::KEYPOINT_SIFT;
        if (keypointDetector == "SIFT") {
            config.PointCloudPostProcess.KeypointDetector = PointCloud::KEYPOINT_SIFT;
        }
        else if (keypointDetector == "ISS_3D") {
            config.PointCloudPostProcess.KeypointDetector = PointCloud::KEYPOINT_ISS_3D;
        }

        // feature detector parsed into enum
        const std::string featureDetector = pointCloudPostProcessConfig["feature_detector"];
        config.PointCloudPostProcess.FeatureDetector = PointCloud::FEATURE_DETECTOR_FPFH;
        if (featureDetector == "FPFH") {
            config.PointCloudPostProcess.FeatureDetector = PointCloud::FEATURE_DETECTOR_FPFH;
        }
        else if (featureDetector == "SHOT_COLOR") {
            config.PointCloudPostProcess.FeatureDetector = PointCloud::FEATURE_DETECTOR_SHOT_COLOR;
        }

        // point cloud feature detection
        nlohmann::json featureDetectionConfig = json["config"]["point_cloud_feature_detection"];
        config.PointCloudFeatureDetection.Normals.Radius = featureDetectionConfig["normals"]["radius"];
        config.PointCloudFeatureDetection.FPFH.MinRadius = featureDetectionConfig["FPFH"]["min_radius"];

        // Point cloud keypoint detection
        nlohmann::json keypointDetectionConfig = json["config"]["point_cloud_keypoint_detection"];
        config.PointCloudKeypointDetection.SIFT.MinScale = keypointDetectionConfig["SIFT"]["min_scale"];
        config.PointCloudKeypointDetection.SIFT.NumOctaves = keypointDetectionConfig["SIFT"]["num_octaves"];
        config.PointCloudKeypointDetection.SIFT.NumScalesPerOctave = keypointDetectionConfig["SIFT"]["num_scales_per_octave"];
        config.PointCloudKeypointDetection.SIFT.MinContrast = keypointDetectionConfig["SIFT"]["min_contrast"];

        // TODO: Parse SHOT and ISS-3D after adding config file support

        return config;
    }
}