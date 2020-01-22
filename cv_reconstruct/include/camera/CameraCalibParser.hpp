//
// CameraCalibParser.hpp
// Parses camera calibration config file from JSON
//

#ifndef CV_RECONSTRUCT_CAMERACALIBPARSER_HPP
#define CV_RECONSTRUCT_CAMERACALIBPARSER_HPP

#include <string>
#include <eigen3/Eigen/Eigen>

#include "nlohmann/json.hpp"
#include "camera/CameraSettings.hpp"

namespace Camera
{
    class CameraCalibParser
    {
    public:
        /// Parse the JSON at the given file path and create a stereo calibration
        /// \param filePath The path to the JSON file with the stereo calibration
        /// \param stereoCalib The stereo calibration (not rectified) that will be populated
        /// \return True if file is read and parsed correctly
        bool ParseStereoCalibJSONFile(const std::string& filePath, Settings::StereoCameraSettings& stereoCalib) const;

    private:
        Eigen::Matrix3f ParseK(const nlohmann::json& json) const;
        Eigen::VectorXf ParseDistortionCoeffs(const nlohmann::json& json) const;
        Eigen::Vector3f ParseT(const nlohmann::json& json) const;
        Eigen::Matrix3f ParseMxN(const nlohmann::json& json, int m, int n) const;
    };
}

#endif //CV_RECONSTRUCT_CAMERACALIBPARSER_HPP
