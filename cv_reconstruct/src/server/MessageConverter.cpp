//
// MessageConverter.cpp
// Converts networking messages to reconstruction records
//

#include "server/MessageConverter.hpp"

#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace Utility
{
    Camera::Calib::StereoCalib MessageConverter::CovertCalibMessage(const CVNetwork::Message::StereoCalibMessage &calibMessage)
    {
        Camera::Calib::StereoCalib calib{};

        calib.LeftCameraCalib.K(0, 0) = calibMessage.fx1;
        calib.LeftCameraCalib.K(1, 1) = calibMessage.fy1;
        calib.LeftCameraCalib.K(0, 2) = calibMessage.cx1;
        calib.LeftCameraCalib.K(1, 2) = calibMessage.cy1;

        calib.LeftCameraCalib.D(0) = calibMessage.d11;
        calib.LeftCameraCalib.D(0) = calibMessage.d12;
        calib.LeftCameraCalib.D(0) = calibMessage.d13;
        calib.LeftCameraCalib.D(0) = calibMessage.d14;
        calib.LeftCameraCalib.D(0) = calibMessage.d15;
        calib.LeftCameraCalib.D(0) = calibMessage.d16;
        calib.LeftCameraCalib.D(0) = calibMessage.d17;
        calib.LeftCameraCalib.D(0) = calibMessage.d18;

        calib.RightCameraCalib.K(0, 0) = calibMessage.fx2;
        calib.RightCameraCalib.K(1, 1) = calibMessage.fy2;
        calib.RightCameraCalib.K(0, 2) = calibMessage.cx2;
        calib.RightCameraCalib.K(1, 2) = calibMessage.cy2;

        calib.RightCameraCalib.D(0) = calibMessage.d21;
        calib.RightCameraCalib.D(0) = calibMessage.d22;
        calib.RightCameraCalib.D(0) = calibMessage.d23;
        calib.RightCameraCalib.D(0) = calibMessage.d24;
        calib.RightCameraCalib.D(0) = calibMessage.d25;
        calib.RightCameraCalib.D(0) = calibMessage.d26;
        calib.RightCameraCalib.D(0) = calibMessage.d27;
        calib.RightCameraCalib.D(0) = calibMessage.d28;

        calib.T(0) = calibMessage.t1;
        calib.T(1) = calibMessage.t2;
        calib.T(2) = calibMessage.t3;

        calib.R(0, 0) = calibMessage.r1;
        calib.R(0, 1) = calibMessage.r2;
        calib.R(0, 2) = calibMessage.r3;
        calib.R(1, 0) = calibMessage.r4;
        calib.R(1, 1) = calibMessage.r5;
        calib.R(1, 2) = calibMessage.r6;
        calib.R(2, 0) = calibMessage.r7;
        calib.R(2, 1) = calibMessage.r8;
        calib.R(2, 2) = calibMessage.r9;

        return calib;
    }

    Reconstruct::StereoFrame MessageConverter::ConvertStereoMessage(const CVNetwork::Message::StereoMessage &stereoMessage)
    {
        Reconstruct::StereoFrame frame{};

        cv::imdecode(stereoMessage.LeftImageData, cv::IMREAD_COLOR, &frame.LeftImage);
        cv::imdecode(stereoMessage.RightImageData, cv::IMREAD_COLOR, &frame.RightImage);

        frame.Translation(0) = stereoMessage.X;
        frame.Translation(1) = stereoMessage.Y;
        frame.Translation(2) = stereoMessage.Z;

        frame.Rotation(0, 0) = stereoMessage.R1;
        frame.Rotation(0, 1) = stereoMessage.R2;
        frame.Rotation(0, 2) = stereoMessage.R3;
        frame.Rotation(1, 0) = stereoMessage.R4;
        frame.Rotation(1, 1) = stereoMessage.R5;
        frame.Rotation(1, 2) = stereoMessage.R6;
        frame.Rotation(2, 0) = stereoMessage.R7;
        frame.Rotation(2, 1) = stereoMessage.R8;
        frame.Rotation(2, 2) = stereoMessage.R9;

        return frame;
    }
}
