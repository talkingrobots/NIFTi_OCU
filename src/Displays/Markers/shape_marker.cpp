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

#include "shape_marker.h"
#include "Displays/MarkerDisplay.h"

#include <ogre_tools/shape.h>

namespace rviz
{

ShapeMarker::ShapeMarker(MarkerDisplay* owner, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
: MarkerBase(owner, sceneMgr, frameTransformer, parent_node)
, shape_(0)
{
}

ShapeMarker::~ShapeMarker()
{
  delete shape_;
}

void ShapeMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  if (!shape_ || old_message->type != new_message->type)
  {
    delete shape_;
    shape_ = 0;

    switch ( new_message->type )
    {
    case visualization_msgs::Marker::CUBE:
      {
        shape_ = new ogre_tools::Shape(ogre_tools::Shape::Cube, sceneMgr, parent_node_);
      }
      break;

    case visualization_msgs::Marker::CYLINDER:
      {
        shape_ = new ogre_tools::Shape(ogre_tools::Shape::Cylinder, sceneMgr, parent_node_);
      }
      break;

    case visualization_msgs::Marker::SPHERE:
      {
        shape_ = new ogre_tools::Shape(ogre_tools::Shape::Sphere, sceneMgr, parent_node_);
      }
      break;

    default:
      ROS_BREAK();
      break;
    }

  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  if (new_message->scale.x * new_message->scale.y * new_message->scale.z == 0.0f)
  {
    owner_->setMarkerStatus(getID(), eu::nifti::ocu::STATUS_LEVEL_WARNING, "Scale of 0 in one of x/y/z");
  }

  shape_->setPosition(pos);
  shape_->setOrientation(orient);
  shape_->setScale(scale);
  shape_->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
}

}
