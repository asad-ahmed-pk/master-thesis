//
// tests_common.cpp
// Common functions used by all test programs
//

#include <opencv2/highgui/highgui.hpp>

#include "tests_common.hpp"

// Get calib and config file
bool GetCalibAndConfig(Camera::Calib::StereoCalib& calib, Config::Config& config)
{
    // parse calib from file
    Camera::CameraCalibParser parser;
    if (!parser.ParseStereoCalibJSONFile(CALIB_FILE, calib)) {
        std::cerr << "\nFailed to parse calib file" << std::endl;
        return false;
    }

    // parse config file
    Config::ConfigParser configParser;
    config = configParser.ParseConfig();

    return true;
}

// Convert to stereo frame
Pipeline::StereoFrame ConvertToFrame(const LocalizationData& data)
{
    Pipeline::StereoFrame frame{};
    frame.Translation = Eigen::Vector3f(data.lat, data.lon, data.alt);
    frame.Rotation = RotationMatrixFromEuler(data.pitch, data.yaw, data.roll);
    return std::move(frame);
}

// Get 3x3 rotation matrix from euler angles (Z - Forward, X - Right, Y - Up)
Eigen::Matrix3f RotationMatrixFromEuler(float pitch, float yaw, float roll)
{
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotation = q.matrix();

    return rotation;
}

// Visualise point cloud (source: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php)
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->registerPointPickingCallback(PointClicked);

    return (viewer);
}

// Mouse click event
void PointClicked(const pcl::visualization::PointPickingEvent& event)
{
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "\nPoint #" << event.getPointIndex();
    std::cout << "\nClicked point at: (" << x << ", " << y << ", " << z << ")";
}

// Read data from localization data file
void ReadLocalizationData(const std::string& filename, std::vector<LocalizationData>& data)
{
    // read in from file
    std::ifstream fs(filename, std::ios::in);
    std::string line;

    LocalizationData localization{};

    // read in localization data from test flat file
    while (std::getline(fs, line))
    {
        // get 6D pose from each line
        std::istringstream is(line);
        is >> localization.lat >> localization.lon >> localization.alt >> localization.roll >> localization.pitch >> localization.yaw;
        data.push_back(localization);
    }
}

// Convert all localization data to stereo frames
void ConvertLocalizationsToStereoFrames(const std::vector<LocalizationData>& data, std::vector<Pipeline::StereoFrame>& frames)
{
    int i = 0;
    std::transform(data.begin(), data.end(), std::back_inserter(frames), ConvertToFrame);
    for (Pipeline::StereoFrame& frame : frames)
    {
        frame.ID = i++;
        frame.LeftImage = cv::imread(std::to_string(frame.ID) + "l.png", cv::IMREAD_COLOR);
        frame.RightImage = cv::imread(std::to_string(frame.ID) + "r.png", cv::IMREAD_COLOR);
    }
}
