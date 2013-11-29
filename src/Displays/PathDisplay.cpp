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

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <tf/transform_listener.h>

#include "ogre_tools/arrow.h"

#include "FloatValidator.h"

#include "Displays/PathDisplay.h"
#include "Displays/Transformers/FrameTransformer.h"

namespace rviz
{
    
    PathDisplay::PathDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
    , topic_(topic)
    , color_(0.1f, 1.0f, 0.0f, 1.0f) // RGB, alpha
    , messages_received_(0)
    , tf_filter_(*frameTransformer->getTFClient(), "", 10, threaded_nh_)
    {
        scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

        static int count = 0;
        std::stringstream ss;
        ss << "Path" << count++;
        manual_object_ = sceneMgr->createManualObject(ss.str());
        manual_object_->setDynamic(true);
        scene_node_->attachObject(manual_object_);

        subscribe();

        tf_filter_.connectInput(sub_);
        tf_filter_.registerCallback(boost::bind(&PathDisplay::incomingMessage, this, _1));
        frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);
    }

    PathDisplay::~PathDisplay()
    {
        unsubscribe();
        clear();

        sceneMgr->destroyManualObject(manual_object_);
        sceneMgr->destroySceneNode(scene_node_->getName());
    }

    void PathDisplay::clear()
    {
        manual_object_->clear();

        messages_received_ = 0;
        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
    }

    void PathDisplay::setColor(const Ogre::ColourValue& color)
    {
        color_ = color;

        processMessage(current_message_);
        causeRender();
    }

    void PathDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        sub_.subscribe(update_nh_, topic_, 10);
    }

    void PathDisplay::unsubscribe()
    {
        sub_.unsubscribe();
    }

    void PathDisplay::onEnable()
    {
        scene_node_->setVisible(true);
        subscribe();
    }

    void PathDisplay::onDisable()
    {
        unsubscribe();
        clear();
        scene_node_->setVisible(false);
    }

    void PathDisplay::fixedFrameChanged()
    {
        clear();

        tf_filter_.setTargetFrame(fixed_frame_);
    }

    void PathDisplay::update(float wall_dt, float ros_dt)
    {
    }

    void PathDisplay::processMessage(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!msg)
        {
            return;
        }

        ++messages_received_;

        if (!FloatValidator::validateFloats(*msg))
        {
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Message contained invalid floating point values (nans or infs)");
            return;
        }

        {
            std::stringstream ss;
            ss << messages_received_ << " messages received";
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }

        manual_object_->clear();

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!frameTransformer->getTransform(msg->header, position, orientation))
        {
            ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str());
        }

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        manual_object_->clear();

        uint32_t num_points = msg->poses.size();
        manual_object_->estimateVertexCount(num_points);
        manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
        for (uint32_t i = 0; i < num_points; ++i)
        {
            Ogre::Vector3 pos(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
            manual_object_->position(pos);
            manual_object_->colour(color_);
        }

        manual_object_->end();
    }

    void PathDisplay::incomingMessage(const nav_msgs::Path::ConstPtr& msg)
    {
        processMessage(msg);
    }

    void PathDisplay::reset()
    {
        Display::reset();
        clear();
    }
    

} // namespace rviz

