//
// MessageConverter.hpp
// Converts networking messages to reconstruction records
//

#ifndef MASTER_THESIS_MESSAGECONVERTER_HPP
#define MASTER_THESIS_MESSAGECONVERTER_HPP

#include "cv_networking/message/StereoStreamMessages.hpp"
#include "camera/CameraCalib.hpp"
#include "pipeline/StereoFrame.hpp"

namespace Utility
{
    class MessageConverter
    {
    public:
        /// Convert the given stereo calibration message
        /// \param calibMessage The calibration message
        /// \return The converted message
        static Camera::Calib::StereoCalib CovertCalibMessage(const CVNetwork::Message::StereoCalibMessage& calibMessage);

        static Pipeline::StereoFrame ConvertStereoMessage(const CVNetwork::Message::StereoMessage& stereoMessage);
    };
}

#endif //MASTER_THESIS_MESSAGECONVERTER_HPP
