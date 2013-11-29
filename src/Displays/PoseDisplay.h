/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


#ifndef RVIZ_POSE_DISPLAY_H_
#define RVIZ_POSE_DISPLAY_H_

#include "Displays/Display.h"

#include <OgreColourValue.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace ogre_tools
{
    class Arrow;
}

namespace Ogre
{
    class SceneNode;
}

namespace rviz
{

    /**
     * \class PoseDisplay
     * \brief Accumulates and displays the pose from a geometry_msgs::PoseStamped message
     */
    class PoseDisplay : public Display
    {
    public:

        PoseDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        virtual ~PoseDisplay();

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt, float ros_dt);
        virtual void reset();

    protected:
        void subscribe();
        void unsubscribe();
        void clear();

        void incomingMessage(const geometry_msgs::PoseStamped::ConstPtr& message);

        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        ogre_tools::Arrow* arrow_;

        uint32_t messages_received_;

        Ogre::SceneNode* scene_node_;

        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_;
        tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter_;
        geometry_msgs::PoseStampedConstPtr latest_message_;
        
        static const float HEAD_RADIUS;
        static const float HEAD_LENGTH;
        static const float SHAFT_RADIUS;
        static const float SHAFT_LENGTH;
        
        static const Ogre::ColourValue COLOR;
    };

} // namespace rviz

#endif /* RVIZ_POSE_DISPLAY_H_ */
