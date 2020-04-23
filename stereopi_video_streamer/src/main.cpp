//
// Stereo Pi Video Streamer
// Streams mp4 file frames from the stereo pi setup described in thesis paper
// Parses OpenCV calibration file in YAML format
//

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "cv_networking/message/StereoStreamMessages.hpp"
#include "cv_networking/client/StereoStreamerClient.hpp"

// File Paths
const std::string CALIB_FILE_LEFT { "calibration_camera_240_left.yml" };
const std::string CALIB_FILE_RIGHT { "calibration_camera_240_right.yml" };
const std::string CALIB_FILE_STEREO { "stereo_camera_calibration240.yml" };

// Server
const std::string SERVER_IP { "127.0.0.1" };
const int SERVER_PORT { 7000 };

// Frame Skip
long FRAME_SKIP_N { 10 };

// Rectification
struct RectificationCalib {
    cv::Mat K1, K2;
    cv::Mat d1, d2;
    cv::Size ImageSize;
    cv::Mat LeftMapX, LeftMapY, RightMapX, RightMapY;
};

// Parse calibration data into calib message to send to server
CVNetwork::Message::StereoCalibMessage ParseCalib(RectificationCalib& rectification);

// Parse video file and extract frames into stereo message
void ParseVideoFile(const std::string& file, std::vector<CVNetwork::Message::StereoMessage>& messages, const RectificationCalib& rectification);

// Parse video frame into stereo message
CVNetwork::Message::StereoMessage ParseVideoFrame(const cv::Mat& frame, const RectificationCalib& rectification);

int main(int argc, char** argv)
{
    // video file - first param
    std::string videoFile { "vid1.mp4" };
    if (argc > 1) {
        std::cout << "\nUsing video file: " << argv[1] << std::endl;
        videoFile = std::string(argv[1]);
    }
    
    // parse calib
    RectificationCalib rectification;
    CVNetwork::Message::StereoCalibMessage calib = ParseCalib(rectification);
    
    // parse video file
    std::vector<CVNetwork::Message::StereoMessage> messages;
    ParseVideoFile(videoFile, messages, rectification);
    
    // first frame is dark - remove
    messages.erase(messages.begin());
    
    // connect to server and stream frames
    CVNetwork::Clients::StereoStreamerClient client(calib);
    if (client.ConnectToReconstructServer(SERVER_IP, SERVER_PORT))
    {
        std::cout << "\nConnected to reconstruction server" << std::endl;

        // run client to begin stereo stream
        client.Run();

        // Add each message to the client's streaming queue
        long id = 0;
        for (const auto& message : messages) {
            std::cout << "\nSending stereo frame to server: " << id++;
            client.AddStereoDataToQueue(message);
        }
    }
    else {
        std::cout << "\nUnable to connect to server at " << SERVER_IP << ": " << SERVER_PORT << std::endl;
        return 1;
    }

    std::cout << "\nMain thread has added all samples to queue. Ctrl-C to end the program" << std::endl;
    while (true) {};
    
    return 0;
}

// Calib parse into message
CVNetwork::Message::StereoCalibMessage ParseCalib(RectificationCalib& rectification)
{
    CVNetwork::Message::StereoCalibMessage message{};
    
    // parse left camera calib
    cv::FileStorage fs1(CALIB_FILE_LEFT, cv::FileStorage::READ);
    cv::Mat K1; cv::Mat d1;
    fs1["camera_matrix"] >> K1;
    fs1["distortion_coeff"] >> d1;
    fs1.release();
    
    std::cout << "\nK1:\n" << K1;
    
    // parse right camera calib
    cv::FileStorage fs2(CALIB_FILE_RIGHT, cv::FileStorage::READ);
    cv::Mat K2; cv::Mat d2;
    fs2["camera_matrix"] >> K2;
    fs2["distortion_coeff"] >> d2;
    fs2.release();
    
    rectification.K1 = K1;
    rectification.K2 = K2;
    rectification.d1 = d1;
    rectification.d2 = d2;
    
    // parse stereo calib
    // parse left camera calib
    cv::FileStorage fs(CALIB_FILE_STEREO, cv::FileStorage::READ);
    cv::Mat t; cv::Mat R;
    fs["translation"] >> t;
    fs["rotation"] >> R;
    
    fs["leftMapX"] >> rectification.LeftMapX;
    fs["rightMapX"] >> rectification.RightMapX;
    fs["leftMapY"] >> rectification.LeftMapY;
    fs["rightMapY"] >> rectification.RightMapY;
    fs["imageSize"] >> rectification.ImageSize;
    
    fs.release();
    
    // create calib message
    message.fx1 = K1.at<double>(0, 0);
    message.fy1 = K1.at<double>(1, 1);
    message.cx1 = K1.at<double>(0, 2);
    message.cy1 = K1.at<double>(1, 2);
    
    message.d11 = d1.at<double>(0, 0);
    message.d12 = d1.at<double>(0, 1);
    message.d13 = d1.at<double>(0, 2);
    message.d14 = d1.at<double>(0, 3);
    
    message.fx2 = K2.at<double>(0, 0);
    message.fy2 = K2.at<double>(1, 1);
    message.cx2 = K2.at<double>(0, 2);
    message.cy2 = K2.at<double>(1, 2);
    
    message.d21 = d2.at<double>(0, 0);
    message.d22 = d2.at<double>(0, 1);
    message.d23 = d2.at<double>(0, 2);
    message.d24 = d2.at<double>(0, 3);
    
    message.r1 = R.at<double>(0, 0);
    message.r2 = R.at<double>(0, 1);
    message.r3 = R.at<double>(0, 2);
    message.r4 = R.at<double>(1, 0);
    message.r5 = R.at<double>(1, 1);
    message.r6 = R.at<double>(1, 2);
    message.r7 = R.at<double>(2, 0);
    message.r8 = R.at<double>(2, 1);
    message.r9 = R.at<double>(2, 2);
    
    message.t1 = t.at<double>(0, 0);
    message.t2 = t.at<double>(0, 1);
    message.t3 = t.at<double>(0, 2);

    return message;
}

// Parse video file and extract frames into stereo message
void ParseVideoFile(const std::string& file, std::vector<CVNetwork::Message::StereoMessage>& messages, const RectificationCalib& rectification)
{
    // open video IO
    cv::VideoCapture capture(file);
    if (capture.isOpened())
    {
        long count = 0;
        while (true)
        {
            cv::Mat frame;
            capture >> frame;
            
            if (frame.empty()) {
                break;
            }
            
            if ((count % FRAME_SKIP_N) == 0) {
                messages.emplace_back(std::move(ParseVideoFrame(frame, rectification)));
            }
            
            count++;
        }
        
        std::cout << count << " frames processed";
        std::cout << "\n" << messages.size() << " messages prepared to send to server" << std::endl;
    }
    else {
        std::cerr << "\nFailed to open video file" << std::endl;
    }
}

// Parse video frame into stereo message
CVNetwork::Message::StereoMessage ParseVideoFrame(const cv::Mat& frame, const RectificationCalib& rectification)
{
    // Note: cameras were configured wrong way round - left is right and vice-versa
    
    CVNetwork::Message::StereoMessage message{};
    
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(640, 240), cv::INTER_CUBIC);

    cv::Mat imgR = cv::Mat(resized, cv::Rect(0, 0, 320, 240));
    cv::Mat imgL = cv::Mat(resized, cv::Rect(320, 0, 320, 240));

    // Rectifying left and right images
    cv::remap(imgL, imgL, rectification.RightMapX, rectification.RightMapY, cv::INTER_LINEAR);
    cv::remap(imgR, imgR, rectification.LeftMapX, rectification.LeftMapY, cv::INTER_LINEAR);
    
    static int id = 0;
    cv::imwrite("frame" + std::to_string(id) + ".png", frame);
    cv::imwrite("frame_left_" + std::to_string(id) + ".png", imgL);
    cv::imwrite("frame_right_" + std::to_string(id++) + ".png", imgR);
    
    cv::imencode(".png", imgL, message.LeftImageData);
    cv::imencode(".png", imgR, message.RightImageData);
        
    return std::move(message);
}
