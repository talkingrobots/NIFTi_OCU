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

#include "Tools/PoseTool.h"

#include "Panels/MultiVizMouseEvent.h"

#include "ogre_tools/camera_base.h"
#include "ogre_tools/arrow.h"
#include "ogre_tools/wx_ogre_render_window.h"
#include "IVisualizationManager.h"
#include "Displays/Transformers/FrameTransformer.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <wx/event.h>

namespace rviz
{

    PoseTool::PoseTool(int8_t id, const std::string& name, const std::string& iconFileName, Ogre::SceneManager* sceneMgr)
    : Tool(id, name, iconFileName)
    {
        arrow_ = new ogre_tools::Arrow(sceneMgr, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
        arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        arrow_->getSceneNode()->setVisible(false);
    }

    PoseTool::~PoseTool()
    {
        delete arrow_;
    }

    void PoseTool::activate()
    {
        state_ = Position;
    }

    void PoseTool::deactivate()
    {
        arrow_->getSceneNode()->setVisible(false);
    }

    bool PoseTool::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
    {
        return false;
    }

    bool PoseTool::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
    {
        if (evt.evt.LeftDown())
        {
            ROS_ASSERT(state_ == Position);

            pos_ = getOgrePositionFromMouseCoordinates(evt.vizMgr->getViewport(), evt.evt.GetX(), evt.evt.GetY());
            arrow_->setPosition(pos_);

            state_ = Orientation;
        }
        else if (evt.evt.Dragging())
        {
            if (state_ == Orientation)
            {
                Ogre::Vector3 cur_pos = getOgrePositionFromMouseCoordinates(evt.vizMgr->getViewport(), evt.evt.GetX(), evt.evt.GetY());
                //                double angle = atan2(pos_.z - cur_pos.z, cur_pos.x - pos_.x);
                //
                //                arrow_->getSceneNode()->setVisible(true);
                //
                //                Ogre::Quaternion base_orient = Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::NEGATIVE_UNIT_Z);
                //                arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle - Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) * base_orient);

                double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

                arrow_->getSceneNode()->setVisible(true);

                //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
                Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

                arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
            }
        }
        else if (evt.evt.LeftUp())
        {
            if (state_ == Orientation)
            {
                Ogre::Vector3 cur_pos = getOgrePositionFromMouseCoordinates(evt.vizMgr->getViewport(), evt.evt.GetX(), evt.evt.GetY());

                Ogre::Vector3 robot_pos = pos_;

                const std::string& fixed_frame = evt.vizMgr->getFixedFrame();
                tf::Stamped<tf::Point> cur_pos_transformed(tf::Point(cur_pos.x, cur_pos.y, cur_pos.z), ros::Time(), fixed_frame);
                tf::Stamped<tf::Point> robot_pos_transformed(tf::Point(robot_pos.x, robot_pos.y, robot_pos.z), ros::Time(), fixed_frame);
                double angle = atan2(cur_pos_transformed.y() - robot_pos_transformed.y(), cur_pos_transformed.x() - robot_pos_transformed.x());

                onPoseSet(robot_pos_transformed.x(), robot_pos_transformed.y(), angle, fixed_frame);

                // These two lines reset the tool and allow sending multiple destination goals one after the other
                this->deactivate();
                this->activate();
            }
        }

        return false;
    }

    //    int PoseTool::processMouseEvent(MultiVizMouseEvent& event)
    //    {
    //        int flags = 0;
    //
    //        if (event.evt.LeftDown())
    //        {
    //            ROS_ASSERT(state_ == Position);
    //
    //            pos_ = getPositionFromMouseXY(event.vizMgr->getViewport(), event.evt.GetX(), event.evt.GetY());
    //            arrow_->setPosition(pos_);
    //
    //            state_ = Orientation;
    //            flags |= Render;
    //        } else if (event.evt.Dragging())
    //        {
    //            if (state_ == Orientation)
    //            {
    //                Ogre::Vector3 cur_pos = getPositionFromMouseXY(event.vizMgr->getViewport(), event.evt.GetX(), event.evt.GetY());
    //                double angle = atan2(pos_.z - cur_pos.z, cur_pos.x - pos_.x);
    //
    //                arrow_->getSceneNode()->setVisible(true);
    //
    //                Ogre::Quaternion base_orient = Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::NEGATIVE_UNIT_Z);
    //                arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle - Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) * base_orient);
    //
    //                flags |= Render;
    //            }
    //        } else if (event.evt.LeftUp())
    //        {
    //            if (state_ == Orientation)
    //            {
    //                Ogre::Vector3 cur_pos = getPositionFromMouseXY(event.vizMgr->getViewport(), event.evt.GetX(), event.evt.GetY());
    //                OgreRobotTransformer::ogreToRobot(cur_pos);
    //
    //                Ogre::Vector3 robot_pos = pos_;
    //                OgreRobotTransformer::ogreToRobot(robot_pos);
    //
    //                const std::string& fixed_frame = event.vizMgr->getFixedFrame();
    //                tf::Stamped<tf::Point> cur_pos_transformed(tf::Point(cur_pos.x, cur_pos.y, cur_pos.z), ros::Time(), fixed_frame);
    //                tf::Stamped<tf::Point> robot_pos_transformed(tf::Point(robot_pos.x, robot_pos.y, robot_pos.z), ros::Time(), fixed_frame);
    //                double angle = atan2(cur_pos_transformed.y() - robot_pos_transformed.y(), cur_pos_transformed.x() - robot_pos_transformed.x());
    //
    //                onPoseSet(robot_pos_transformed.x(), robot_pos_transformed.y(), angle, fixed_frame);
    //
    //                flags |= (Finished | Render);
    //            }
    //        }
    //
    //        return flags;
    //    }

}

