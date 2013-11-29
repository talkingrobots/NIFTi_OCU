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


#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Displays/MarkerDisplay.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "marker_base.h"

namespace rviz
{

MarkerBase::MarkerBase(MarkerDisplay* owner, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
: owner_(owner)
, sceneMgr(sceneMgr)
, frameTransformer(frameTransformer)
, parent_node_(parent_node->createChildSceneNode())
{}

MarkerBase::~MarkerBase()
{
  sceneMgr->destroySceneNode(parent_node_);
}

void MarkerBase::setMessage(const MarkerConstPtr& message)
{
  MarkerConstPtr old = message_;
  message_ = message;

  expiration_ = ros::Time::now() + message->lifetime;

  onNewMessage(old, message);
}

void MarkerBase::updateFrameLocked()
{
  ROS_ASSERT(message_ && message_->frame_locked);
  onNewMessage(message_, message_);
}

bool MarkerBase::expired()
{
  return ros::Time::now() >= expiration_;
}

bool MarkerBase::transform(const MarkerConstPtr& message, Ogre::Vector3& pos, Ogre::Quaternion& orient, Ogre::Vector3& scale)
{
  ros::Time stamp = message->header.stamp;
  if (message->frame_locked)
  {
    stamp = ros::Time();
  }

  if (!frameTransformer->transform(message->header.frame_id, stamp, message->pose, pos, orient))
  {
    std::string error;
    frameTransformer->transformHasProblems(message->header.frame_id, message->header.stamp, error);
    owner_->setMarkerStatus(getID(), eu::nifti::ocu::STATUS_LEVEL_ERROR, error);
    return false;
  }

  scale = Ogre::Vector3(message->scale.x, message->scale.y, message->scale.z);

  return true;
}

} // namespace rviz
