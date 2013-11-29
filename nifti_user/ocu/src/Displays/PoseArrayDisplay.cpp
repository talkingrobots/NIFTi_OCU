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

#include "ogre_tools/arrow.h"

#include "FloatValidator.h"

#include <tf/transform_listener.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/PoseArrayDisplay.h"

namespace rviz
{

    const Ogre::ColourValue PoseArrayDisplay::COLOR(1.0f, 0.1f, 0.0f, 1.0f);
    
PoseArrayDisplay::PoseArrayDisplay( const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue )
: Display( name, sceneMgr, frameTransformer, updateQueue, threadQueue )
, messages_received_(0)
, tf_filter_(*frameTransformer->getTFClient(), "", 2, update_nh_)
{
  this->topic = topic;    
    
  scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "ParticleCloud2D" << count++;
  manual_object_ = sceneMgr->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PoseArrayDisplay::incomingMessage, this, _1));
  frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);
}

PoseArrayDisplay::~PoseArrayDisplay()
{
  unsubscribe();
  clear();

  sceneMgr->destroyManualObject( manual_object_ );
}

void PoseArrayDisplay::clear()
{
  manual_object_->clear();

  messages_received_ = 0;
  setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
}

void PoseArrayDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic, 5);
}

void PoseArrayDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PoseArrayDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void PoseArrayDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PoseArrayDisplay::fixedFrameChanged()
{
  clear();
  tf_filter_.setTargetFrame( fixed_frame_ );
}

void PoseArrayDisplay::update(float wall_dt, float ros_dt)
{
}

void PoseArrayDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ++messages_received_;

  if (!FloatValidator::validateFloats(*msg))
  {
    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR ,"Message contained invalid floating point values (nans or infs)");
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
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  size_t num_poses = msg->poses.size();
  manual_object_->estimateVertexCount( num_poses * 6 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( size_t i=0; i < num_poses; ++i)
  {
    Ogre::Vector3 pos(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->poses[i].orientation, quat);
    Ogre::Quaternion orient = Ogre::Quaternion::IDENTITY;
    orient = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orient;

    const static float radius = 0.3f;
    Ogre::Vector3 vertices[6];
    vertices[0] = pos; // back of arrow
    vertices[1] = pos + orient * Ogre::Vector3(radius, 0, 0); // tip of arrow
    vertices[2] = vertices[1];
    vertices[3] = pos + orient * Ogre::Vector3(0.75*radius, 0.2*radius, 0);
    vertices[4] = vertices[1];
    vertices[5] = pos + orient * Ogre::Vector3(0.75*radius, -0.2*radius, 0);

    for ( int i = 0; i < 6; ++i )
    {
      manual_object_->position( vertices[i] );
      manual_object_->colour( COLOR );
    }
  }
  manual_object_->end();

  causeRender();
}

void PoseArrayDisplay::incomingMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  processMessage(msg);
}

void PoseArrayDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

