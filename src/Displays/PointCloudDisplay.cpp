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

#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/PointCloudDisplay.h"

namespace rviz
{

PointCloudDisplay::PointCloudDisplay( const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue )
: PointCloudBase( name, sceneMgr, frameTransformer, updateQueue, threadQueue )
, tf_filter_(*frameTransformer->getTFClient(), "", 10, threaded_nh_)
{
  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(&PointCloudDisplay::incomingCloudCallback, this);
  frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);

  // Temporary configuration (doing some tests)
  setAlpha(1);
  setStyle(ogre_tools::PointCloud::RM_POINTS); // 0: Points  1: Billboards
  //setBillboardSize(0.1);
  setColorTransformer("Intensity");
  setDecayTime(60);
  setXYZTransformer("XYZ");
  setTopic("/slam_cloud");
  
}

PointCloudDisplay::~PointCloudDisplay()
{
  unsubscribe();
}

void PointCloudDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  this->topic = topic;
  reset();
  subscribe();

  causeRender();
}

void PointCloudDisplay::onEnable()
{
  PointCloudBase::onEnable();

  subscribe();
}

void PointCloudDisplay::onDisable()
{
  unsubscribe();
  tf_filter_.clear();

  PointCloudBase::onDisable();
}

void PointCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(threaded_nh_, topic, 10);
}

void PointCloudDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PointCloudDisplay::incomingCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud)
{
  addMessage(cloud);
}

void PointCloudDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame( fixed_frame_ );

  PointCloudBase::fixedFrameChanged();
}

} // namespace rviz
