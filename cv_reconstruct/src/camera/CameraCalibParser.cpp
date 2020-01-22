//
// CameraCalibParser.cpp
// Parses camera calibration config file from JSON
//

#include "camera/CameraCalibParser.hpp"

#include <vector>
#include <boost/filesystem.hpp>

namespace Camera
{
    // JSON root node
    const std::string ROOT_NODE { "stereo_calib" };

    // Parse calib in JSON file
    bool CameraCalibParser::ParseStereoCalibJSONFile(const std::string& filePath, Settings::StereoCameraSettings& stereoCalib) const
    {
        if (!boost::filesystem::exists(filePath)) {
            return false;
        }

        std::ifstream fs { filePath };

        // read in json from file stream
        nlohmann::json json;
        fs >> json;
        fs.close();

        // parse intrinsics for left camera
        stereoCalib.LeftCamSettings.K = ParseK(json[ROOT_NODE]["intrinsic"]["left_cam"]["K"]);
        stereoCalib.LeftCamSettings.D = ParseDistortionCoeffs(json[ROOT_NODE]["intrinsic"]["left_cam"]["distortion"]);

        // parse intrinsics for right camera
        stereoCalib.RightCamSettings.K = ParseK(json[ROOT_NODE]["intrinsic"]["right_cam"]["K"]);
        stereoCalib.RightCamSettings.D = ParseDistortionCoeffs(json[ROOT_NODE]["intrinsic"]["right_cam"]["distortion"]);

        // parse extrinsics (relative transform b/w cameras with 1st cam as world origin)
        stereoCalib.T = ParseT(json[ROOT_NODE]["extrinsic"]["T"]);
        stereoCalib.R = ParseMxN(json[ROOT_NODE]["extrinsic"]["R"], 3, 3);

        return true;
    }

    Eigen::Matrix3f CameraCalibParser::ParseK(const nlohmann::json& json) const
    {
        Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
        K << json["fx"], 0.0, json["cx"], 0.0, json["fy"], json["cy"], 0.0, 0.0, 1.0;
        return std::move(K);
    }

    Eigen::VectorXf CameraCalibParser::ParseDistortionCoeffs(const nlohmann::json& json) const
    {
        std::vector<float> values = json;
        Eigen::VectorXf d{ values.size() };

        for (int i = 0; i < values.size(); i++) {
            d(i) = values[i];
        }

        return std::move(d);
    }

    Eigen::Vector3f CameraCalibParser::ParseT(const nlohmann::json& json) const
    {
        Eigen::Vector3f T = Eigen::Vector3f::Zero();
        T << json[0], json[1], json[2];
        return std::move(T);
    }

    // Parse regular MxN matrix
    Eigen::Matrix3f CameraCalibParser::ParseMxN(const nlohmann::json& json, int m, int n) const
    {
        Eigen::Matrix3f A = Eigen::Matrix3f::Zero();

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                A(i, j) = json[i][j];
            }
        }
        return std::move(A);
    }
}
