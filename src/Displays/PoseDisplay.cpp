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

#include "ogre_tools/arrow.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include "FloatValidator.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/PoseDisplay.h"


namespace rviz
{
    
    const float PoseDisplay::HEAD_RADIUS = 0.5;
    const float PoseDisplay::HEAD_LENGTH = 0.25;
    const float PoseDisplay::SHAFT_RADIUS = 0.25;
    const float PoseDisplay::SHAFT_LENGTH = 0.25;
    
    const Ogre::ColourValue PoseDisplay::COLOR(1.0f, 0.1f, 0.0f, 1.0f);

    PoseDisplay::PoseDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
    , messages_received_(0)
    , tf_filter_(*frameTransformer->getTFClient(), "", 5, update_nh_)
    {
        this->topic = topic;
        
        scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

        tf_filter_.connectInput(sub_);
        tf_filter_.registerCallback(boost::bind(&PoseDisplay::incomingMessage, this, _1));
        frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);

        arrow_ = new ogre_tools::Arrow(sceneMgr, scene_node_, SHAFT_LENGTH, SHAFT_RADIUS, HEAD_LENGTH, HEAD_RADIUS);
        arrow_->setColor(COLOR);

        subscribe();
    }

    PoseDisplay::~PoseDisplay()
    {
        unsubscribe();

        clear();

        delete arrow_;
    }

    void PoseDisplay::clear()
    {
        tf_filter_.clear();
        latest_message_.reset();

        messages_received_ = 0;
        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
    }

    void PoseDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        sub_.subscribe(update_nh_, this->topic, 5);
    }

    void PoseDisplay::unsubscribe()
    {
        sub_.unsubscribe();
    }

    void PoseDisplay::onEnable()
    {
        scene_node_->setVisible(true);

        subscribe();
    }

    void PoseDisplay::onDisable()
    {
        unsubscribe();
        clear();
        scene_node_->setVisible(false);
    }

    void PoseDisplay::fixedFrameChanged()
    {
        tf_filter_.setTargetFrame(fixed_frame_);
        clear();
    }

    void PoseDisplay::update(float wall_dt, float ros_dt)
    {
    }

    void PoseDisplay::incomingMessage(const geometry_msgs::PoseStamped::ConstPtr& message)
    {
        ++messages_received_;

        if (!FloatValidator::validateFloats(*message))
        {
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Message contained invalid floating point values (nans or infs)");
            return;
        }

        {
            std::stringstream ss;
            ss << messages_received_ << " messages received";
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK,ss.str());
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!frameTransformer->transform(message->header, message->pose, position, orientation))
        {
            ROS_ERROR("Error transforming pose '%s' from frame '%s' to frame '%s'", name.c_str(), message->header.frame_id.c_str(), fixed_frame_.c_str());
        }

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        latest_message_ = message;

        causeRender();
    }

    void PoseDisplay::reset()
    {
        Display::reset();
        clear();
    }

} // namespace rviz
