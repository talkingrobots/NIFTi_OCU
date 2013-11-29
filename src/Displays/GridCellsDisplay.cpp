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
#include <ogre_tools/point_cloud.h>

#include "Displays/Transformers/FrameTransformer.h"
#include "FloatValidator.h"

#include "Displays/GridCellsDisplay.h"

namespace rviz
{
    const std::string GridCellsDisplay::ROS_TOPIC = "/costmap_2d/costmap/obstacles";

GridCellsDisplay::GridCellsDisplay( const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue )
: Display( name, sceneMgr, frameMgr, updateQueue, threadQueue )
, color( 0.1f, 1.0f, 0.0f, 1.0f )
, tf_filter_(*frameMgr->getTFClient(), "", 10, update_nh_)
, messages_received_(0)
{
  scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new ogre_tools::PointCloud();
  cloud_->setRenderMode( ogre_tools::PointCloud::RM_BILLBOARDS_COMMON_FACING );
  cloud_->setCommonDirection( Ogre::Vector3::UNIT_Y );
  cloud_->setCommonUpVector( Ogre::Vector3::NEGATIVE_UNIT_Z );
  scene_node_->attachObject(cloud_);

  setTopic(ROS_TOPIC);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&GridCellsDisplay::incomingMessage, this, _1));
  frameMgr->registerFilterForTransformStatusCheck(tf_filter_, this);
}

GridCellsDisplay::~GridCellsDisplay()
{
  unsubscribe();
  clear();

  sceneMgr->destroySceneNode(scene_node_->getName());
  delete cloud_;
}

void GridCellsDisplay::clear()
{
  cloud_->clear();

  messages_received_ = 0;
  setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
}

void GridCellsDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  this->topic = topic;

  subscribe();

  causeRender();
}

    void GridCellsDisplay::setColor(const Ogre::ColourValue& color)
    {
        this->color = color;

        processMessage(current_message_);
        causeRender();
    }

    const Ogre::ColourValue& GridCellsDisplay::getColor() const
    {
        return color;
    }

void GridCellsDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic, 10);
}

void GridCellsDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void GridCellsDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void GridCellsDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void GridCellsDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void GridCellsDisplay::update(float wall_dt, float ros_dt)
{
}

bool validateFloats(const nav_msgs::GridCells& msg)
{
  bool valid = true;
  valid = valid && FloatValidator::validateFloats(msg.cell_width);
  valid = valid && FloatValidator::validateFloats(msg.cell_height);
  valid = valid && FloatValidator::validateFloats(msg.cells);
  return valid;
}

void GridCellsDisplay::processMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  cloud_->clear();

  ++messages_received_;

  if (!validateFloats(*msg))
  {
    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Message contained invalid floating point values (nans or infs)");
    return;
  }

  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!frameTransformer->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0);

  uint32_t num_points = msg->cells.size();

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( num_points );
  for(uint32_t i = 0; i < num_points; i++)
  {
    ogre_tools::PointCloud::Point& current_point = points[ i ];

    Ogre::Vector3 pos(msg->cells[i].x, msg->cells[i].y, msg->cells[i].z);

    current_point.x = pos.x;
    current_point.y = pos.y;
    current_point.z = pos.z;
    current_point.setColor(color.r, color.g, color.b);
  }

  cloud_->clear();

  if ( !points.empty() )
  {
    cloud_->addPoints( &points.front(), points.size() );
  }
}

void GridCellsDisplay::incomingMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  processMessage(msg);
}

void GridCellsDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

