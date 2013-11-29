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

#ifndef RVIZ_FRAME_MANAGER_H
#define RVIZ_FRAME_MANAGER_H

#include <map>

#include <ros/time.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Pose.h>
//#include <roslib/Header.h>

#include <tf/message_filter.h>

namespace tf
{
    class TransformListener;
}

namespace rviz
{
    class Display;

    //class FrameTransformer;
    //typedef boost::shared_ptr<FrameTransformer> FrameManagerPtr;
    //typedef boost::weak_ptr<FrameTransformer> FrameManagerWPtr;

    class FrameTransformer
    {
    public:
        //static FrameManagerPtr instance();

        FrameTransformer();
        ~FrameTransformer();

        void setFixedFrame(const std::string& frame);

        template<typename Header>
        bool getTransform(const Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation)
        {
            return getTransform(header.frame_id, header.stamp, position, orientation);
        }

        bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation);

        template<typename Header>
        bool transform(const Header& header, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation) const
        {
            return transform(header.frame_id, header.stamp, pose, position, orientation);
        }

        bool transform(const std::string& frame, const ros::Time& time, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation) const;
        
        static bool transform(const std::string& fixedFrame, const std::string& frame, Ogre::Vector3& position, Ogre::Quaternion& orientation);
        static bool transform(const std::string& fixedFrame, const std::string& frame, const ros::Time&, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation);
        
        void update();

        bool frameHasProblems(const std::string& frame, ros::Time time, std::string& error) const;
        bool transformHasProblems(const std::string& frame, ros::Time time, std::string& error) const;

        template<class M>
        void registerFilterForTransformStatusCheck(tf::MessageFilter<M>& filter, Display* display) const
        {
            filter.registerCallback(boost::bind(&FrameTransformer::messageCallback<M>, this, _1, display));
            filter.registerFailureCallback(boost::bind(&FrameTransformer::failureCallback<M>, this, _1, _2, display));
        }

        const std::string& getFixedFrame() const;

        static tf::TransformListener* getTFClient();

        std::string discoverFailureReason(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason) const;

    private:

        template<class M>
        void messageCallback(const boost::shared_ptr<M const>& msg, Display* display) const
        {
            messageArrived(msg->header.frame_id, msg->header.stamp, msg->__connection_header ? (*msg->__connection_header)["callerid"] : "unknown", display);
        }

        template<class M>
        void failureCallback(const boost::shared_ptr<M const>& msg, tf::FilterFailureReason reason, Display* display) const
        {
            messageFailed(msg->header.frame_id, msg->header.stamp, msg->__connection_header ? (*msg->__connection_header)["callerid"] : "unknown", reason, display);
        }

        void messageArrived(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, Display* display) const;
        void messageFailed(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason, Display* display) const;

        struct CacheKey
        {

            CacheKey(const std::string& f, const ros::Time & t)
            : frame(f)
            , time(t)
            {
            }

            bool operator<(const CacheKey & rhs) const
            {
                if (frame != rhs.frame)
                {
                    return frame < rhs.frame;
                }

                return time < rhs.time;
            }

            const std::string frame;
            const ros::Time time;
        };

        struct CacheEntry
        {

            CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion & o)
            : position(p)
            , orientation(o)
            {
            }

            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
        };
        typedef std::map<CacheKey, CacheEntry > MapOfTFCaches;

        boost::mutex cache_mutex_;
        MapOfTFCaches cache_;

        static tf::TransformListener* tf_; // Benoit: I made this static so that multiple frame transformers share the same TF Listener
        std::string fixed_frame_;
    };

}

#endif // RVIZ_FRAME_MANAGER_H
