// SfM test

#define CERES_FOUND 1

#include <iostream>

#include <eigen3/Eigen/Eigen>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/sfm/reconstruct.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define fx 959.791015
#define fy 956.925109
#define cx 696.021728
#define cy 224.180603

using namespace std;
using namespace cv;
using namespace cv::sfm;

int main(int argc, char** argv)
{
    // Parse the image paths
    vector<String> images_paths;
    for (int i = 0; i < 16; i++) {
        images_paths.push_back(std::string("keyframe_" + std::to_string(i) + ".png"));
    }
    
    // Build intrinsics
    Matx33d K = Matx33d( fx, 0, cx,
                         0, fy, cy,
                         0, 0,  1);
    bool is_projective = true;
    vector<Mat> Rs_est, ts_est, points3d_estimated;
    reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);
    
    // Print output
    cout << "\n----------------------------\n" << endl;
    cout << "Reconstruction: " << endl;
    cout << "============================" << endl;
    cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
    cout << "Estimated cameras: " << Rs_est.size() << endl;
    cout << "Refined intrinsics: " << endl << K << endl << endl;
    cout << "3D Visualization: " << endl;
    cout << "============================" << endl;
    
    viz::Viz3d window("Coordinate Frame");
               window.setWindowSize(Size(500,500));
               window.setWindowPosition(Point(150,150));
               window.setBackgroundColor(); // black by default
    
    // Create the pointcloud
    cout << "Recovering points  ... ";
    // recover estimated points3d
    vector<Vec3f> point_cloud_est;
    for (int i = 0; i < points3d_estimated.size(); ++i)
      point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
    cout << "[DONE]" << endl;
    cout << "Recovering cameras ... ";
    vector<Affine3d> path;
    for (size_t i = 0; i < Rs_est.size(); ++i)
      path.push_back(Affine3d(Rs_est[i],ts_est[i]));
    cout << "[DONE]" << endl;
    if ( point_cloud_est.size() > 0 )
    {
      cout << "Rendering points   ... ";
      viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
      window.showWidget("point_cloud", cloud_widget);
      cout << "[DONE]" << endl;
    }
    else
    {
      cout << "Cannot render points: Empty pointcloud" << endl;
    }
    if ( path.size() > 0 )
    {
      cout << "Rendering Cameras  ... ";
      window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
      window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
      window.setViewerPose(path[0]);
      cout << "[DONE]" << endl;
    }
    else
    {
      cout << "Cannot render the cameras: Empty path" << endl;
    }
    cout << endl << "Press 'q' to close each windows ... " << endl;
    window.spin();
    
    return 0;
}
