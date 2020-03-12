//
// tests_common.cpp
// Common functions used by all test programs
//

#include <opencv2/highgui/highgui.hpp>

#include "tests_common.hpp"

#define EARTH_RADIUS_METERS 6378137.0f

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

// Create a point with RGB and XYZ
pcl::PointXYZRGB CreatePoint(float x, float y, float z, int r, int g, int b)
{
    pcl::PointXYZRGB p{};

    p.x = x; p.y = y; p.z = z;
    p.r = r; p.g = g; p.b = b;

    return p;
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

// Convert point to text coords
std::string PointCoordsToString(const pcl::PointXYZRGB& point) {
    return ("(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ", " + std::to_string(point.z) + ")");
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

// Add 4 point point cloud
void AddCameraDebugShape(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int r, int g, int b, float scale)
{
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            pcl::PointXYZRGB p(r, g, b);

            p.x = scale * i;
            p.y = scale * j;
            p.z = scale;

            cloud.push_back(p);
        }
    }

    // line to plane as camera (forward line)
    for (int i = 0; i < scale; i++)
    {
        pcl::PointXYZRGB p(0, 0, 255);
        p.x = p.y = 0; p.z = i;
        cloud.push_back(p);
    }

    // line upwards
    for (int i = 0; i < scale; i++)
    {
        pcl::PointXYZRGB p(0, 255, 255);
        p.x = 0; p.y = i; p.z = 0;
        cloud.push_back(p);
    }
}

// RGB to greyscale
void ConvertToGreyScale(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointXYZI>& result)
{
    pcl::PointXYZI p;
    for (int i = 0; i < input.points.size(); i++)
    {
        Eigen::Vector3i rgb = input.points[i].getRGBVector3i();

        float r = rgb(0) / 255.0f;
        float g = rgb(1) / 255.0f;
        float b = rgb(2) / 255.0f;

        float intensity = (0.3f * r + 0.59f * g + 0.11f * b);
        p = pcl::PointXYZI(intensity);

        p.x = input.points[i].x;
        p.y = input.points[i].y;
        p.z = input.points[i].z;

        result.push_back(p);
    }
}

// GPS to Mercator Projection
// X: Right, Y: Up, Z: Forward
Eigen::Vector3f ProjectGPSToMercator(float latitude, float longitude, float altitude)
{
    // record the mercator scale
    static float mercatorScale = -1.0;
    if (mercatorScale < 0.0) {
        mercatorScale = cosf((latitude * static_cast<float>(M_PI)) / 180.0f);
    }

    Eigen::Vector3f T = Eigen::Vector3f::Zero();

    float fwd = mercatorScale * longitude * M_PI * EARTH_RADIUS_METERS / 180.0;
    float left = mercatorScale * EARTH_RADIUS_METERS * log(tan((M_PI * (90 + latitude)) / 360.0));
    float up = altitude;

    T(0) = -left;
    T(1) = up;
    T(2) = fwd;

#ifndef NDEBUG
    std::cout << "\nGPS:\n" << T << std::endl;
#endif

    return std::move(T);
}

// Get world pose for the stereo frame in euclidean coordinate space
Eigen::Matrix4f GetFrameWorldPose(const Pipeline::StereoFrame& frame)
{
    // get mercator projection
    Eigen::Vector3f t = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2));

    // store first transform
    static std::unique_ptr<Eigen::Matrix4f> T0 { nullptr };
    if (T0 == nullptr)
    {
        T0 = std::make_unique<Eigen::Matrix4f>(Eigen::Matrix4f::Identity());
        T0->block(0, 0, 3, 3) = frame.Rotation;
        T0->block(0, 3, 3, 1) = t;
    }

    // this transform
    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    T1.block(0, 0, 3, 3) = frame.Rotation;
    T1.block(0, 3, 3, 1) = t - T0->block(0, 3, 3, 1);

    return T1;

    // convert frame to coordinate system relative to origin (world space)
    //Eigen::Matrix4f T1_W = T0->inverse() * T1;

    //return T1_W;
}
