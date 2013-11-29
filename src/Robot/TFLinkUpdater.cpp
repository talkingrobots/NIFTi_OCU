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

#include "Robot/TFLinkUpdater.h"
#include "Displays/Transformers/FrameTransformer.h"

#include <tf/tf.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

TFLinkUpdater::TFLinkUpdater(FrameTransformer* frameTransformer, const StatusCallback& status_cb, const std::string& tf_prefix)
: frame_manager_(frameTransformer)
, status_callback_(status_cb)
, tf_prefix_(tf_prefix)
{
}

bool TFLinkUpdater::getLinkTransforms(const std::string& _link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                      Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation, bool& apply_offset_transforms) const
{
  std::string link_name = _link_name;
  if (!tf_prefix_.empty())
  {
    link_name = tf::resolve(tf_prefix_, link_name);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!frame_manager_->getTransform(link_name, ros::Time(), position, orientation))
  {
    std::stringstream ss;
    ss << "No transform from [" << link_name << "] to [" << frame_manager_->getFixedFrame() << "]";
    setLinkStatus(link_name, eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
    return false;
  }

  setLinkStatus(link_name, eu::nifti::ocu::STATUS_LEVEL_OK, "Transform OK");

  // Collision/visual transforms are the same in this case
  visual_position = position;
  visual_orientation = orientation;
  collision_position = position;
  collision_orientation = orientation;
  apply_offset_transforms = true;

  return true;
}

void TFLinkUpdater::setLinkStatus(const std::string& link_name, eu::nifti::ocu::StatusLevel level, const std::string& text) const
{
  if (status_callback_)
  {
    status_callback_(level, link_name, text);
  }
}

}
