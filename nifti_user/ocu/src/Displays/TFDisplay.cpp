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


#include "Displays/FrameInfo.h"

#include "Displays/Transformers/FrameTransformer.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include "Displays/TFDisplay.h"

namespace rviz
{

// This is the Biba config at ETH
//TF.All\ Enabled=1
//TF.Enabled=1
//TF.Frame\ Timeout=15
//TF.Show\ Arrows=1
//TF.Show\ Axes=1
//TF.Show\ Names=1
//TF.Update\ Interval=0

static const Ogre::ColourValue ARROW_HEAD_COLOR(1.0f, 0.1f, 0.6f, 1.0f);
static const Ogre::ColourValue ARROW_SHAFT_COLOR(0.8f, 0.8f, 0.3f, 1.0f);
    
void TFDisplay::setFrameEnabled(FrameInfo* frame, bool enabled)
{
  frame->enabled_ = enabled;

  if (frame->name_node_)
  {
    frame->name_node_->setVisible(show_names_ && enabled);
  }

  if (frame->axes_)
  {
    frame->axes_->getSceneNode()->setVisible(show_axes_ && enabled);
  }

  if (frame->parent_arrow_)
  {
    if (frame->distance_to_parent_ > 0.001f)
    {
      frame->parent_arrow_->getSceneNode()->setVisible(show_arrows_ && enabled);
    }
    else
    {
      frame->parent_arrow_->getSceneNode()->setVisible(false);
    }
  }

  if (all_enabled_ && !enabled)
  {
    all_enabled_ = false;
  }

  causeRender();
}

TFDisplay::TFDisplay( const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue )
: Display( name, sceneMgr, frameMgr, updateQueue, threadQueue )
, update_timer_( 0.0f )
, update_rate_( 0.0f )
, show_names_( true )
, show_arrows_( true )
, show_axes_( true )
, frame_timeout_(15.0f)
, all_enabled_(true)
{
  root_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

  names_node_ = root_node_->createChildSceneNode();
  arrows_node_ = root_node_->createChildSceneNode();
  axes_node_ = root_node_->createChildSceneNode();

  // This is the Biba config at ETH
  setAllEnabled(true); //TF.All\ Enabled=1


}

TFDisplay::~TFDisplay()
{
  clear();

  root_node_->removeAndDestroyAllChildren();
  sceneMgr->destroySceneNode( root_node_->getName() );
}

void TFDisplay::clear()
{
  std::set<FrameInfo*> to_delete;
  M_FrameInfo::iterator frame_it = frames_.begin();
  M_FrameInfo::iterator frame_end = frames_.end();
  for ( ; frame_it != frame_end; ++frame_it )
  {
    to_delete.insert( frame_it->second );
  }

  std::set<FrameInfo*>::iterator delete_it = to_delete.begin();
  std::set<FrameInfo*>::iterator delete_end = to_delete.end();
  for ( ; delete_it != delete_end; ++delete_it )
  {
    deleteFrame( *delete_it );
  }

  frames_.clear();

  update_timer_ = 0.0f;

  clearStatuses();
}

void TFDisplay::onEnable()
{
  root_node_->setVisible( true );

  names_node_->setVisible( show_names_ );
  arrows_node_->setVisible( show_arrows_ );
  axes_node_->setVisible( show_axes_ );
}

void TFDisplay::onDisable()
{
  root_node_->setVisible( false );
  clear();
}

void TFDisplay::setShowNames( bool show )
{
  show_names_ = show;

  names_node_->setVisible( show );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::setShowAxes( bool show )
{
  show_axes_ = show;

  axes_node_->setVisible( show );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::setShowArrows( bool show )
{
  show_arrows_ = show;

  arrows_node_->setVisible( show );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::setAllEnabled(bool enabled)
{
  all_enabled_ = enabled;

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, enabled);
  }
}

void TFDisplay::setFrameTimeout(float timeout)
{
  frame_timeout_ = timeout;
}

void TFDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;
}

void TFDisplay::update(float wall_dt, float ros_dt)
{
  update_timer_ += wall_dt;
  if ( update_rate_ < 0.0001f || update_timer_ > update_rate_ )
  {
    updateFrames();

    update_timer_ = 0.0f;
  }
}

FrameInfo* TFDisplay::getFrameInfo( const std::string& frame )
{
  M_FrameInfo::iterator it = frames_.find( frame );
  if ( it == frames_.end() )
  {
    return NULL;
  }

  return it->second;
}

void TFDisplay::updateFrames()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  frameTransformer->getTFClient()->getFrameStrings( frames );
  std::sort(frames.begin(), frames.end());

  std::set<FrameInfo*> current_frames;

  {
    V_string::iterator it = frames.begin();
    V_string::iterator end = frames.end();
    for ( ; it != end; ++it )
    {
      const std::string& frame = *it;

      if ( frame.empty() )
      {
        continue;
      }

      FrameInfo* info = getFrameInfo( frame );
      if (!info)
      {
        info = createFrame(frame);
      }
      else
      {
        updateFrame(info);
      }

      current_frames.insert( info );
    }
  }

  {
    std::set<FrameInfo*> to_delete;
    M_FrameInfo::iterator frame_it = frames_.begin();
    M_FrameInfo::iterator frame_end = frames_.end();
    for ( ; frame_it != frame_end; ++frame_it )
    {
      if ( current_frames.find( frame_it->second ) == current_frames.end() )
      {
        to_delete.insert( frame_it->second );
      }
    }

    std::set<FrameInfo*>::iterator delete_it = to_delete.begin();
    std::set<FrameInfo*>::iterator delete_end = to_delete.end();
    for ( ; delete_it != delete_end; ++delete_it )
    {
      deleteFrame( *delete_it );
    }
  }

  causeRender();
}



FrameInfo* TFDisplay::createFrame(const std::string& frame)
{
  FrameInfo* info = new FrameInfo;
  frames_.insert( std::make_pair( frame, info ) );

  info->name_ = frame;
  info->last_update_ = ros::Time::now();
  info->axes_ = new ogre_tools::Axes( sceneMgr, axes_node_, 0.2, 0.02 );
  info->axes_->getSceneNode()->setVisible( show_axes_ );

  info->name_text_ = new ogre_tools::MovableText( frame, "Arial", 0.1 );
  info->name_text_->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject( info->name_text_ );
  info->name_node_->setVisible( show_names_ );

  info->parent_arrow_ = new ogre_tools::Arrow( sceneMgr, arrows_node_, 1.0f, 0.01, 1.0f, 0.08 );
  info->parent_arrow_->getSceneNode()->setVisible( false );
  info->parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
  info->parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);

  info->enabled_ = true;

  updateFrame( info );

  return info;
}

Ogre::ColourValue lerpColor(const Ogre::ColourValue& start, const Ogre::ColourValue& end, float t)
{
  return start * t + end * (1 - t);
}

void TFDisplay::updateFrame(FrameInfo* frame)
{
  tf::TransformListener* tf = frameTransformer->getTFClient();

  // Check last received time so we can grey out/fade out frames that have stopped being published
  ros::Time latest_time;
  tf->getLatestCommonTime(fixed_frame_, frame->name_, latest_time, 0);
  if (latest_time != frame->last_time_to_fixed_)
  {
    frame->last_update_ = ros::Time::now();
    frame->last_time_to_fixed_ = latest_time;
  }

  // Fade from color -> grey, then grey -> fully transparent
  ros::Duration age = ros::Time::now() - frame->last_update_;
  float one_third_timeout = frame_timeout_ * 0.3333333f;
  if (age > ros::Duration(frame_timeout_))
  {
    frame->parent_arrow_->getSceneNode()->setVisible(false);
    frame->axes_->getSceneNode()->setVisible(false);
    frame->name_node_->setVisible(false);
    return;
  }
  else if (age > ros::Duration(one_third_timeout))
  {
    Ogre::ColourValue grey(0.7, 0.7, 0.7, 1.0);

    if (age > ros::Duration(one_third_timeout * 2))
    {
      float a = std::max(0.0, (frame_timeout_ - age.toSec())/one_third_timeout);
      Ogre::ColourValue c = Ogre::ColourValue(grey.r, grey.g, grey.b, a);

      frame->axes_->setXColor(c);
      frame->axes_->setYColor(c);
      frame->axes_->setZColor(c);
      frame->name_text_->setColor(c);
      frame->parent_arrow_->setColor(c.r, c.g, c.b, c.a);
    }
    else
    {
      float t = std::max(0.0, (one_third_timeout * 2 - age.toSec())/one_third_timeout);
      frame->axes_->setXColor(lerpColor(frame->axes_->getDefaultXColor(), grey, t));
      frame->axes_->setYColor(lerpColor(frame->axes_->getDefaultYColor(), grey, t));
      frame->axes_->setZColor(lerpColor(frame->axes_->getDefaultZColor(), grey, t));
      frame->name_text_->setColor(lerpColor(Ogre::ColourValue::White, grey, t));
      frame->parent_arrow_->setShaftColor(lerpColor(ARROW_SHAFT_COLOR, grey, t));
      frame->parent_arrow_->setHeadColor(lerpColor(ARROW_HEAD_COLOR, grey, t));
    }
  }
  else
  {
    frame->axes_->setToDefaultColors();
    frame->name_text_->setColor(Ogre::ColourValue::White);
    frame->parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
    frame->parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);
  }

  setStatus(frame->name_, eu::nifti::ocu::STATUS_LEVEL_OK, "Transform OK");

  if (!frameTransformer->getTransform(frame->name_, ros::Time(), frame->position_, frame->orientation_))
  {
    std::stringstream ss;
    ss << "No transform from [" << frame->name_ << "] to frame [" << fixed_frame_ << "]";
    setStatus(frame->name_, eu::nifti::ocu::STATUS_LEVEL_WARNING, ss.str());
    ROS_DEBUG("Error transforming frame '%s' to frame '%s'", frame->name_.c_str(), fixed_frame_.c_str());
  }

  frame->robot_space_position_ = frame->position_;

  frame->robot_space_orientation_ = frame->orientation_;

  frame->axes_->setPosition( frame->position_ );
  frame->axes_->setOrientation( frame->orientation_ );
  frame->axes_->getSceneNode()->setVisible(show_axes_ && frame->enabled_);

  frame->name_node_->setPosition( frame->position_ );
  frame->name_node_->setVisible(show_names_ && frame->enabled_);

  std::string old_parent = frame->parent_;
  frame->parent_.clear();
  bool has_parent = tf->getParent( frame->name_, ros::Time(), frame->parent_ );
  if ( has_parent )
  {


    if ( show_arrows_ )
    {
      Ogre::Vector3 parent_position;
      Ogre::Quaternion parent_orientation;
      if (!frameTransformer->getTransform(frame->parent_, ros::Time(), parent_position, parent_orientation))
      {
        ROS_DEBUG("Error transforming frame '%s' (parent of '%s') to frame '%s'", frame->parent_.c_str(), frame->name_.c_str(), fixed_frame_.c_str());
      }

      Ogre::Vector3 direction = parent_position - frame->position_;
      float distance = direction.length();
      direction.normalise();

      Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );

      Ogre::Vector3 old_pos = frame->parent_arrow_->getPosition();

      bool distance_changed = fabsf(distance - frame->distance_to_parent_) > 0.0001f;
      if ( distance_changed )
      {
        frame->distance_to_parent_ = distance;
        float head_length = ( distance < 0.1 ) ? (0.1*distance) : 0.1;
        float shaft_length = distance - head_length;
        frame->parent_arrow_->set( shaft_length, 0.02, head_length, 0.08 );
      }

      if ( distance > 0.001f )
      {
        frame->parent_arrow_->getSceneNode()->setVisible( show_arrows_ && frame->enabled_ );
      }
      else
      {
        frame->parent_arrow_->getSceneNode()->setVisible( false );
      }

      frame->parent_arrow_->setPosition( frame->position_ );
      frame->parent_arrow_->setOrientation( orient );
    }
    else
    {
      frame->parent_arrow_->getSceneNode()->setVisible( false );
    }
  }
  else
  {
    frame->parent_arrow_->getSceneNode()->setVisible( false );
  }
}

void TFDisplay::deleteFrame(FrameInfo* frame)
{
  M_FrameInfo::iterator it = frames_.find( frame->name_ );
  ROS_ASSERT( it != frames_.end() );

  frames_.erase( it );

  delete frame->axes_;
  delete frame->parent_arrow_;
  delete frame->name_text_;
  sceneMgr->destroySceneNode( frame->name_node_->getName() );
  delete frame;
}

void TFDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

