//
// KITTIVisionParser.hpp
// KITTI vision dataset parser. Parses the data files and loads calib and image data.
//

#ifndef KITTI_VISION_STREAMER_KITTIVISIONPARSER_HPP
#define KITTI_VISION_STREAMER_KITTIVISIONPARSER_HPP

#include <vector>
#include <string>

#include "dataset.hpp"

class KITTIVisionParser
{
public:
    /// Create the dataset with the given folder paths
    /// \param calibFolder The folder with the calib file 'calib_cam_to_cam.txt'
    /// \param dataFolder The folder with the image folders 'image_02', 'image_03', and 'oxts'
    KITTIVisionParser(std::string calibFolder, std::string dataFolder);

    ~KITTIVisionParser() = default;

    /// Get the parsed calibration data
    /// \return The calibration data for the stereo system
    Calib GetParsedCalibData() const;

    /// Get the parsed data samples
    /// \param samples Will be populated with data samples containing the images and pose
    void GetParsedDataSamples(std::vector<DataSample>& samples) const;

private:
    void ParseCalib();
    void ParseData();
    void ReadLines(const std::string& filePath, std::vector<std::string>& lines) const;
    Eigen::Matrix3f MatrixFromLine(const std::string& line) const;
    Eigen::Vector3f Vector3FromLine(const std::string& line) const;
    Eigen::VectorXf VectorXFromLine(const std::string& line, int n) const;
    Eigen::Matrix3f RotationMatrixFromEuler(float roll, float pitch, float yaw) const;
    void GetFloatComponentsFromLine(const std::string& line, std::vector<float>& components) const;
    void GetFilesInDirectory(const std::string& directory, std::vector<std::string>& files) const;
    DataSample BuildSampleFromFileID(const std::string& fileID) const;

private:
    const std::string m_CalibFolderPath;
    const std::string m_DataFolderPath;
    std::string m_ImageFileExtension;
    std::vector<DataSample> m_DataSamples;
    std::vector<std::string> m_FileIDs;
    Calib m_Calib{};
};

#endif //KITTI_VISION_STREAMER_KITTIVISIONPARSER_HPP
