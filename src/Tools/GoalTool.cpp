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

#include "Tools/GoalTool.h"

#include "ogre_tools/camera_base.h"
#include "ogre_tools/arrow.h"
#include "ogre_tools/wx_ogre_render_window.h"

#include <geometry_msgs/PoseStamped.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include <tf/transform_listener.h>

#include <wx/event.h>

namespace rviz
{

    GoalTool::GoalTool(int8_t id, const std::string& name, const std::string& iconFileName, Ogre::SceneManager* sceneMgr)
    : PoseTool(id, name, iconFileName, sceneMgr)
    {
        setTopic("/move_base_simple/goal"); // Todo Benoit: Put that string in a more appropriate place
        //setTopic("/nav/user_destination"); // Todo Benoit: Put that string in a more appropriate place
    }

    GoalTool::~GoalTool()
    {
    }

    void GoalTool::setTopic(const std::string& topic)
    {
        topic_ = topic;
        pub_ = nh_.advertise<geometry_msgs::PoseStamped > (topic, 1);
    }

    void GoalTool::onPoseSet(double x, double y, double theta, const std::string& fixed_frame)
    {
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose > (tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        
        geometry_msgs::PoseStamped goal;
        tf::poseStampedTFToMsg(p, goal);

//        ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
//                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
//                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
        
        pub_.publish(goal);
    }

}

