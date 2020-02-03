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

#define NUM_COMPONENTS_D 5

// Constructor
KITTIVisionParser::KITTIVisionParser(std::string calibFolder, std::string dataFolder) : m_CalibFolderPath(std::move(calibFolder)), m_DataFolderPath(std::move(dataFolder))
{
    ParseCalib();
}

// Get parsed calib data
Calib KITTIVisionParser::GetParsedCalibData() const {
    return m_Calib;
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

    m_Calib.D1 = VectorXFromLine(lines[K1_LINE_INDEX + 1], NUM_COMPONENTS_D);
    m_Calib.D2 = VectorXFromLine(lines[K2_LINE_INDEX + 1], NUM_COMPONENTS_D);


    m_Calib.T = Vector3FromLine(lines[K2_LINE_INDEX + 3]) - Vector3FromLine(lines[K1_LINE_INDEX + 3]);
    m_Calib.R = MatrixFromLine(lines[K1_LINE_INDEX + 2]) * MatrixFromLine(lines[K2_LINE_INDEX + 2]);
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

// Init dynamic vector x from line
Eigen::VectorXf KITTIVisionParser::VectorXFromLine(const std::string &line, int n) const
{
    Eigen::VectorXf v(n);

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
