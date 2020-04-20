//
// KeyFrameDatabase.cpp
// Shared database for keyframes
//

#include <iostream>
#include <fstream>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>

#include "system/KeyFrameDatabase.hpp"

namespace System
{
    // Insert
    size_t KeyFrameDatabase::InsertKeyFrame(std::shared_ptr<TrackingFrame> frame)
    {
        size_t id = m_NextUsableID++;
        
        m_DatabaseMutex.lock();
        m_KeyFrames[id] = frame;
        frame->SetID(id);
        m_LastInsertedID = id;
        m_DatabaseMutex.unlock();
        
        // save camera image to disk for permanent access
        cv::imwrite("keyframe_" + std::to_string(id) + ".png", frame->GetCameraImage());
        
        std::cout << "\nKey Frame Created: " << "#" << id << std::endl;
        
        return id;
    }

    // Select by ID
    std::shared_ptr<TrackingFrame> KeyFrameDatabase::SelectKeyFrame(size_t id)
    {
        std::shared_ptr<TrackingFrame> frame { nullptr };
        if (m_KeyFrames.find(id) != m_KeyFrames.end()) {
            frame = m_KeyFrames[id];
        }
        
        return frame;
    }

    // Get last recent keyframe
    std::shared_ptr<TrackingFrame> KeyFrameDatabase::SelectMostRecentKeyFrame() {
        return m_KeyFrames[m_LastInsertedID];
    }

    // Get path
    std::string KeyFrameDatabase::GetKeyFrameImagePath(size_t id) const {
        return std::string("keyframe_" + std::to_string(id) + ".png");
    }

    // Update keyframe pose
    void KeyFrameDatabase::UpdateKeyFramePose(size_t id, const Eigen::Matrix4f& pose)
    {
        std::shared_ptr<TrackingFrame> frame = SelectKeyFrame(id);
        
        m_DatabaseMutex.lock();
        if (frame != nullptr) {
            frame->SetTrackedPose(pose);
        }
        m_DatabaseMutex.unlock();
    }

    // Empty
    bool KeyFrameDatabase::IsEmpty() const {
        return m_KeyFrames.empty();
    }

    // Count
    size_t KeyFrameDatabase::GetCount() const {
        return m_KeyFrames.size();
    }

    // Dump to CSV
    void KeyFrameDatabase::DumpPosesToCSV()
    {
        // open file stream
        std::ofstream fs;
        fs.open("estimated_poses.csv", std::ios::out);
        
        if (fs.is_open())
        {
            // header
            fs << "KeyFrame_ID," << "X," << "Y," << "Z," << "Roll," << "Pitch," << "Yaw";
            
            for (size_t i = 0; i < m_KeyFrames.size(); i++)
            {
                if (m_KeyFrames.find(i) != m_KeyFrames.end())
                {
                    auto kf = m_KeyFrames[i];
                    
                    // id
                    size_t id = kf->GetID();
                    
                    // pose
                    Eigen::Affine3f affine;
                    affine.matrix() = kf->GetTrackedPose();
                    
                    // position
                    float x = affine.translation().x();
                    float y = affine.translation().y();
                    float z = affine.translation().z();
                    
                    // rotation
                    Eigen::Matrix3f rotation = affine.rotation();
                    Eigen::Vector3f euler = rotation.eulerAngles(0, 1, 2);
                    
                    float roll = euler(0) * (180.0 / M_PI);
                    float pitch = euler(1) * (180.0 / M_PI);
                    float yaw = euler(2) * (180.0 / M_PI);
                    
                    // write record
                    fs << "\n";
                    fs << id << "," << x << "," << y << "," << z << "," << roll << "," << pitch << "," << yaw;
                }
            }
            
            fs.close();
            std::cout << "\n\nPoses written to disk" << std::endl;
            
        }
        else {
            std::cerr << "\nFailed to write to csv file. Ensure you have space or write access" << std::endl;
        }
    }
}
