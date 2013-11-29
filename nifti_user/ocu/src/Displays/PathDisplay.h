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


#ifndef RVIZ_PATH_DISPLAY_H
#define RVIZ_PATH_DISPLAY_H

#include <OGRE/OgreColourValue.h>

#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include <nav_msgs/Path.h>

#include "Displays/Display.h"

namespace Ogre
{
    class SceneNode;
    class ManualObject;
}

namespace rviz
{

    /**
     * \class PathDisplay
     * \brief Displays a nav_msgs::Path message
     */
    class PathDisplay : public Display
    {
    public:
        PathDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        virtual ~PathDisplay();

        const std::string& getTopic() const
        {
            return topic_;
        }

        void setColor(const Ogre::ColourValue& color);

        const Ogre::ColourValue& getColor() const
        {
            return color_;
        }

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt, float ros_dt);
        virtual void reset();

        

    protected:
        void subscribe();
        void unsubscribe();
        void clear();
        void incomingMessage(const nav_msgs::Path::ConstPtr& msg);
        void processMessage(const nav_msgs::Path::ConstPtr& msg);

        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        std::string topic_;
        Ogre::ColourValue color_;

        uint32_t messages_received_;

        Ogre::SceneNode* scene_node_;
        Ogre::ManualObject* manual_object_;

        message_filters::Subscriber<nav_msgs::Path> sub_;
        tf::MessageFilter<nav_msgs::Path> tf_filter_;
        nav_msgs::Path::ConstPtr current_message_;

    };

} // namespace rviz

#endif /* RVIZ_PATH_DISPLAY_H */

