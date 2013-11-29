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

#ifndef RVIZ_FRAME_SELECTION_HANDLER_H
#define RVIZ_FRAME_SELECTION_HANDLER_H

#include <ogre_tools/arrow.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>

#include <ros/ros.h>

namespace rviz
{
    class TFDisplay;

    struct FrameInfo
    {

        FrameInfo()
        : axes_(NULL)
        , parent_arrow_(NULL)
        , name_text_(NULL)
        , position_(Ogre::Vector3::ZERO)
        , orientation_(Ogre::Quaternion::IDENTITY)
        , distance_to_parent_(0.0f)
        , arrow_orientation_(Ogre::Quaternion::IDENTITY)
        , robot_space_position_(Ogre::Vector3::ZERO)
        , robot_space_orientation_(Ogre::Quaternion::IDENTITY)
        , enabled_(true)
        {

        }

        const Ogre::Vector3 & getPositionInRobotSpace()
        {
            return robot_space_position_;
        }

        const Ogre::Quaternion & getOrientationInRobotSpace()
        {
            return robot_space_orientation_;
        }

        const std::string & getParent()
        {
            return parent_;
        }

        bool isEnabled()
        {
            return enabled_;
        }

        std::string name_;
        std::string parent_;
        ogre_tools::Axes* axes_;
        ogre_tools::Arrow* parent_arrow_;
        ogre_tools::MovableText* name_text_;
        Ogre::SceneNode* name_node_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;
        float distance_to_parent_;
        Ogre::Quaternion arrow_orientation_;

        Ogre::Vector3 robot_space_position_;
        Ogre::Quaternion robot_space_orientation_;

        bool enabled_;

        ros::Time last_update_;
        ros::Time last_time_to_fixed_;
    };

} // namespace rviz

#endif // RVIZ_FRAME_SELECTION_HANDLER_H
