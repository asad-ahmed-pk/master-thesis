//
// KeyFrameDatabase.hpp
// Shared database for keyframes
//

#ifndef KEYFRAME_DATABASE_HPP
#define KEYFRAME_DATABASE_HPP

#include <mutex>
#include <unordered_map>
#include <memory>

#include <eigen3/Eigen/Eigen>

#include "TrackingFrame.hpp"

namespace System
{
    class KeyFrameDatabase
    {
    public:
        /// Construct the default instance of the keyframe database
        KeyFrameDatabase() = default;
        
        ~KeyFrameDatabase() = default;
        
        /// Insert a keyframe into the database
        /// \param frame The keyframe to insert
        /// \return The ID of the newly inserted keyframe
        size_t InsertKeyFrame(std::shared_ptr<TrackingFrame> frame);
        
        /// Get a pointer to en existing keyframe with the given id
        /// \param id The ID of the keyframe
        /// \return The shared pointer to a keyframe if found. Null if not found.
        std::shared_ptr<TrackingFrame> SelectKeyFrame(size_t id);
        
        /// Get the most recent keyframe
        /// \return A shared pointer to the most recent keyframe
        std::shared_ptr<TrackingFrame> SelectMostRecentKeyFrame();
        
        /// Update the pose of the given keyframe
        /// \param id The id of the Keyframe
        /// \param pose The new pose for the keyframe
        void UpdateKeyFramePose(size_t id, const Eigen::Isometry3d& pose);
        
    private:
        size_t m_NextUsableID { 0 };
        size_t m_LastInsertedID { 0 };
        std::unordered_map<size_t, std::shared_ptr<TrackingFrame>> m_KeyFrames;
        std::mutex m_DatabaseMutex;
    };
}

#endif
