//
// KITTIVisionParser.cpp
// KITTI vision dataset parser. Parses the data files and loads calib and image data.
//

#include "KITTIVisionParser.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#define CALIB_FILE_NAME "calib_cam_to_cam.txt"

#define K1_LINE_INDEX 19
#define K2_LINE_INDEX 27
#define CAM_0_R_INDEX 5
#define CAM_0_T_INDEX 6

#define IMAGE_LEFT_FOLDER_NAME "image_02"
#define IMAGE_RIGHT_FOLDER_NAME "image_03"
#define GPS_FOLDER_NAME "oxts"
#define DATA_SUBFOLDER_NAME "data"

#define NUM_COMPONENTS_D 5

// Constructor
KITTIVisionParser::KITTIVisionParser(std::string calibFolder, std::string dataFolder, bool isRectifiedData) : m_CalibFolderPath(std::move(calibFolder)), m_DataFolderPath(std::move(dataFolder)), m_IsRectifiedData(isRectifiedData)
{
    ParseCalib();
    ParseData();
}

// Get parsed calib data
Calib KITTIVisionParser::GetParsedCalibData() const {
    return m_Calib;
}

// Get parsed dataset
void KITTIVisionParser::GetParsedDataSamples(std::vector<DataSample>& samples) const {
    samples = m_DataSamples;
}

// Parse calib
void KITTIVisionParser::ParseCalib()
{
    // build full path to calib txt file
    boost::filesystem::path calibFilepath(m_CalibFolderPath);
    calibFilepath /= CALIB_FILE_NAME;

    // read calib data lines from text file
    std::vector<std::string> lines;
    ReadLines(calibFilepath.string(), lines);

    // parse calib data
    m_Calib.K1 = MatrixFromLine(lines[K1_LINE_INDEX]);
    m_Calib.K2 = MatrixFromLine(lines[K2_LINE_INDEX]);

    m_Calib.D1 = VectorXFromLine(lines[K1_LINE_INDEX + 1], NUM_COMPONENTS_D, 8);
    m_Calib.D2 = VectorXFromLine(lines[K2_LINE_INDEX + 1], NUM_COMPONENTS_D, 8);

    m_Calib.T = Vector3FromLine(lines[K1_LINE_INDEX + 3]) - Vector3FromLine(lines[K2_LINE_INDEX + 3]);

    if (!m_IsRectifiedData) {
        m_Calib.R = MatrixFromLine(lines[K1_LINE_INDEX + 2]).transpose() * MatrixFromLine(lines[K2_LINE_INDEX + 2]);
    }
    else {
        m_Calib.R = MatrixFromLine(lines[K1_LINE_INDEX + 5]).transpose() * MatrixFromLine(lines[K2_LINE_INDEX + 5]);
    }
}

// Parse all data
void KITTIVisionParser::ParseData()
{
    // full path to the left image directory
    boost::filesystem::path leftImagesDirPath(m_DataFolderPath);
    leftImagesDirPath /= IMAGE_LEFT_FOLDER_NAME;
    leftImagesDirPath /= DATA_SUBFOLDER_NAME;

    // get list of files paths for the left images
    std::vector<std::string> files;
    GetFilesInDirectory(leftImagesDirPath.string(), files);

    // no files in this directory
    if (files.empty()) {
        return;
    }

    // extract and store file IDs from these full paths
    std::transform(files.begin(), files.end(), std::back_inserter(m_FileIDs), [](const std::string& f) -> std::string {
        boost::filesystem::path p(f);
        return p.stem().string();
    });

    // sort fileIDs in ascending order
    std::sort(m_FileIDs.begin(), m_FileIDs.end(), [](const std::string& id1, const std::string& id2) {
        long num1 = std::stol(id1);
        long num2 = std::stol(id2);
        return (num1 < num2);
    });

    // store image file extension
    boost::filesystem::path firstFile(files[0]);
    m_ImageFileExtension = firstFile.extension().string();

    // build all samples from this file ID list
    for (const std::string& id : m_FileIDs) {
        m_DataSamples.emplace_back(std::move(BuildSampleFromFileID(id)));
    }
}

// Build data sample from file ID
DataSample KITTIVisionParser::BuildSampleFromFileID(const std::string &fileID) const
{
    DataSample sample;

    sample.ID = fileID;

    // build image 1 and 2 paths
    boost::filesystem::path img1Path(m_DataFolderPath);
    img1Path = (img1Path / IMAGE_LEFT_FOLDER_NAME / DATA_SUBFOLDER_NAME / (fileID + m_ImageFileExtension));
    sample.Camera1ImagePath = img1Path.string();

    boost::filesystem::path img2Path(m_DataFolderPath);
    img2Path = (img2Path / IMAGE_RIGHT_FOLDER_NAME / DATA_SUBFOLDER_NAME / (fileID + m_ImageFileExtension));
    sample.Camera2ImagePath = img2Path.string();

#ifndef NDEBUG
    assert(boost::filesystem::exists(img1Path));
    assert(boost::filesystem::exists(img2Path));
#endif

    // Get pose data from oxts data folder
    boost::filesystem::path oxtFilePath(m_DataFolderPath);
    oxtFilePath = (oxtFilePath / GPS_FOLDER_NAME / DATA_SUBFOLDER_NAME / (fileID + ".txt"));

    // open the oxt text file and get required pose data
    std::ifstream fs(oxtFilePath.string(), std::ios::in);
    float x, y, z, roll, pitch, yaw;

    fs >> y >> x >> z >> roll >> pitch >> yaw;
    fs.close();

    // convert roll, pitch, yaw to rotation matrix
    sample.T = Eigen::Vector3f(x, y, z);
    sample.R = RotationMatrixFromEuler(roll, pitch, yaw);

    return sample;
}

// Read lines from text file
void KITTIVisionParser::ReadLines(const std::string &filePath, std::vector<std::string> &lines) const
{
    std::ifstream fs;
    fs.open(filePath, std::ios::in);

    std::string line;
    while (std::getline(fs, line)) {
        lines.push_back(line);
    }
}

// Get list of files in directory
void KITTIVisionParser::GetFilesInDirectory(const std::string& directory, std::vector<std::string>& files) const
{
    boost::filesystem::directory_iterator end;
    boost::filesystem::path path(directory);

    for (boost::filesystem::directory_iterator iter(path); iter != end; iter++)
    {
        if (boost::filesystem::is_regular_file(iter->path()) && !iter->path().stem().empty()) {
            files.push_back(iter->path().string());
        }
    }
}

// Init 3x3 matrix from line with row-major order components
Eigen::Matrix3f KITTIVisionParser::MatrixFromLine(const std::string &line) const
{
    static constexpr int n = 3;

    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    std::vector<float> components;

    GetFloatComponentsFromLine(line, components);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            M(i, j) = components[n * i + j];
        }
    }

    return M;
}

// Get 3x3 rotation matrix from euler angles (Z - Forward, X - Right, Y - Up)
Eigen::Matrix3f KITTIVisionParser::RotationMatrixFromEuler(float roll, float pitch, float yaw) const
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotation = q.matrix();

    return rotation;
}

// Init 3d vector from line
Eigen::Vector3f KITTIVisionParser::Vector3FromLine(const std::string &line) const
{
    static constexpr int n = 3;

    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    std::vector<float> components;

    GetFloatComponentsFromLine(line, components);

    for (int i = 0; i < n; i++) {
        v(i) = components[i];
    }

    return v;
}

// Init dynamic vector x from line with max size M but with elements until n
Eigen::VectorXf KITTIVisionParser::VectorXFromLine(const std::string &line, int n, int m) const
{
    Eigen::VectorXf v(m);

    std::vector<float> components;
    GetFloatComponentsFromLine(line, components);

    for (int i = 0; i < n; i++) {
        v(i) = components[i];
    }

    return v;
}

// Extract row-major components as float from the given string (separated by space)
void KITTIVisionParser::GetFloatComponentsFromLine(const std::string &line, std::vector<float> &components) const
{
    std::vector<std::string> stringComponents;
    boost::algorithm::split(stringComponents, line, boost::is_space());

    // get rid of first component and transform the rest to floats
    stringComponents.erase(stringComponents.begin());
    std::transform(stringComponents.begin(), stringComponents.end(), std::back_inserter(components), [](const std::string& s) -> float { return std::stof(s); });
}
