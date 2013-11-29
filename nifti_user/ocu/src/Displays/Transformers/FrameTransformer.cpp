/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "Displays/Display.h"

#include <tf/transform_listener.h>
#include <ros/ros.h>

#include "Displays/Transformers/FrameTransformer.h"

namespace rviz
{

    //FrameManagerPtr FrameTransformer::instance()
    //{
    //  static FrameManagerWPtr instw;
    //
    //  FrameManagerPtr inst = instw.lock();
    //  if (!inst)
    //  {
    //    inst.reset(new FrameTransformer);
    //    instw = inst;
    //  }
    //
    //  return inst;
    //}

    tf::TransformListener* FrameTransformer::tf_ = NULL;

    FrameTransformer::FrameTransformer()
    {
        // Initializes the static transform listener
        if (tf_ == NULL)
        {
            tf_ = new tf::TransformListener(ros::NodeHandle(), ros::Duration(10 * 60), false);
        }
    }

    FrameTransformer::~FrameTransformer()
    {
        // Benoit: Not necessary anymore since it is static
        //delete tf_;
    }

    void FrameTransformer::update()
    {
        boost::mutex::scoped_lock lock(cache_mutex_);
        cache_.clear();
    }

    void FrameTransformer::setFixedFrame(const std::string& frame)
    {
        boost::mutex::scoped_lock lock(cache_mutex_);
        fixed_frame_ = frame;
        cache_.clear();
    }

    bool FrameTransformer::getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation)
    {
        boost::mutex::scoped_lock lock(cache_mutex_);

        position = Ogre::Vector3(9999999, 9999999, 9999999);
        orientation = Ogre::Quaternion::IDENTITY;

        if (fixed_frame_.empty())
        {
            return false;
        }

        MapOfTFCaches::iterator it = cache_.find(CacheKey(frame, time));
        if (it != cache_.end())
        {
            position = it->second.position;
            orientation = it->second.orientation;
            return true;
        }

        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0f;

        if (!transform(frame, time, pose, position, orientation))
        {
            return false;
        }

        cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));

        return true;
    }

    /**
     * This version of transform uses the fixed frame in the instance
     * @param frame
     * @param time
     * @param pose_msg
     * @param position
     * @param orientation
     * @return 
     */
    bool FrameTransformer::transform(const std::string& frame, const ros::Time& time, const geometry_msgs::Pose& pose_msg, Ogre::Vector3& position, Ogre::Quaternion& orientation) const
    {
        return transform(fixed_frame_, frame, time, pose_msg, position, orientation);
    }

    /**
     * This static version of transform transforms from fixed frame to frame at the time of the latest known transform
     * @param fixedFrame
     * @param frame
     * @param position
     * @param orientation
     * @return 
     */
    bool FrameTransformer::transform(const std::string& fixedFrame, const std::string& frame, Ogre::Vector3& position, Ogre::Quaternion& orientation)
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0f;
        return transform(fixedFrame, frame, ros::Time(), pose, position, orientation);
    }

    /**
     * This static version of transform is the one that does the calculations
     * @param fixedFrame
     * @param frame
     * @param time
     * @param pose_msg
     * @param position
     * @param orientation
     * @return 
     */
    bool FrameTransformer::transform(const std::string& fixedFrame, const std::string& frame, const ros::Time& time, const geometry_msgs::Pose& pose_msg, Ogre::Vector3& position, Ogre::Quaternion& orientation)
    {
        position = Ogre::Vector3::ZERO;
        orientation = Ogre::Quaternion::IDENTITY;

        // put all pose data into a tf stamped pose
        tf::Quaternion bt_orientation(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
        tf::Vector3 bt_position(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

        if (bt_orientation.x() == 0.0 && bt_orientation.y() == 0.0 && bt_orientation.z() == 0.0 && bt_orientation.w() == 0.0)
        {
            bt_orientation.setW(1.0);
        }

        tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation, bt_position), time, frame);
        tf::Stamped<tf::Pose> pose_out;

        // convert pose into new frame
        try
        {
            tf_->transformPose(fixedFrame, pose_in, pose_out);
        }
        catch (tf::TransformException& e)
        {
            ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(), fixedFrame.c_str(), e.what());
            return false;
        }

        bt_position = pose_out.getOrigin();
        position = Ogre::Vector3(bt_position.x(), bt_position.y(), bt_position.z());

        bt_orientation = pose_out.getRotation();
        orientation = Ogre::Quaternion(bt_orientation.w(), bt_orientation.x(), bt_orientation.y(), bt_orientation.z());

        return true;
    }

    bool FrameTransformer::frameHasProblems(const std::string& frame, ros::Time time, std::string& error) const
    {
        if (!tf_->frameExists(frame))
        {
            error = "Frame [" + frame + "] does not exist";
            if (frame == fixed_frame_)
            {
                error = "Fixed " + error;
            }
            return true;
        }

        return false;
    }

    bool FrameTransformer::transformHasProblems(const std::string& frame, ros::Time time, std::string& error) const
    {
        std::string tf_error;
        bool transform_succeeded = tf_->canTransform(fixed_frame_, frame, time, &tf_error);
        if (transform_succeeded)
        {
            return false;
        }

        bool ok = true;
        ok = ok && !frameHasProblems(fixed_frame_, time, error);
        ok = ok && !frameHasProblems(frame, time, error);

        if (ok)
        {
            std::stringstream ss;
            ss << "No transform to fixed frame [" << fixed_frame_ << "].  TF error: [" << tf_error << "]";
            error = ss.str();
            ok = false;
        }

        {
            std::stringstream ss;
            ss << "For frame [" << frame << "]: " << error;
            error = ss.str();
        }

        return !ok;
    }

    const std::string& FrameTransformer::getFixedFrame() const
    {
        return fixed_frame_;
    }

    tf::TransformListener* FrameTransformer::getTFClient()
    {
        return tf_;
    }

    std::string FrameTransformer::discoverFailureReason(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason) const
    {
        if (reason == tf::filter_failure_reasons::OutTheBack)
        {
            std::stringstream ss;
            ss << "Message removed because it is too old (frame=[" << frame_id << "], stamp=[" << stamp << "])";
            return ss.str();
        }
        else
        {
            std::string error;
            if (transformHasProblems(frame_id, stamp, error))
            {
                return error;
            }
        }

        return "Unknown reason for transform failure";
    }

    std::string getTransformStatusName(const std::string& caller_id)
    {
        std::stringstream ss;
        ss << "Transform [sender=" << caller_id << "]";
        return ss.str();
    }

    void FrameTransformer::messageArrived(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, Display* display) const
    {
        display->setStatus(getTransformStatusName(caller_id), eu::nifti::ocu::STATUS_LEVEL_OK, "Transform OK");
    }

    void FrameTransformer::messageFailed(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason, Display* display) const
    {
        std::string status_name = getTransformStatusName(caller_id);
        std::string status_text = discoverFailureReason(frame_id, stamp, caller_id, reason);
        display->setStatus(status_name, eu::nifti::ocu::STATUS_LEVEL_ERROR, status_text);
    }

}
