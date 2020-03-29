//
// KeyFrameDatabase.cpp
// Shared database for keyframes
//

#include "system/KeyFrameDatabase.hpp"

namespace System
{
    // Insert
    size_t KeyFrameDatabase::InsertKeyFrame(std::shared_ptr<TrackingFrame> frame)
    {
        size_t id = m_NextUsableID++;
        
        m_DatabaseMutex.lock();
        m_KeyFrames[id] = frame;
        m_LastInsertedID = id;
        m_DatabaseMutex.unlock();
        
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

    // Update keyframe pose
    void KeyFrameDatabase::UpdateKeyFramePose(size_t id, const Eigen::Isometry3d& pose)
    {
        std::shared_ptr<TrackingFrame> frame = SelectKeyFrame(id);
        
        m_DatabaseMutex.lock();
        if (frame != nullptr) {
            frame->SetTrackedPose(pose);
        }
        m_DatabaseMutex.unlock();
    }
}
