/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef AXIS_COLOR_PC_TRANSFORMER_H
#define	AXIS_COLOR_PC_TRANSFORMER_H

#include "Displays/Transformers/PointCloud/PointCloudTransformer.h"

//namespace Ogre
//{
//    class Matrix4;
//}

namespace rviz
{

    class AxisColorPCTransformer : public PointCloudTransformer
{
public:
  AxisColorPCTransformer()
    : min_value_(-10.0f)
    , max_value_(10.0f)
    , use_fixed_frame_(true)
    , axis_(AXIS_Z)
    {
      setAutoComputeBounds(true);
    }

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) const;
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
//  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props);
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void setMinValue(float val);
  void setMaxValue(float val);
  float getMinValue() { return min_value_; }
  float getMaxValue() { return max_value_; }
  void setAutoComputeBounds(bool compute);
  bool getAutoComputeBounds() { return auto_compute_bounds_; }

  void setUseFixedFrame(bool use);
  bool getUseFixedFrame() { return use_fixed_frame_; }

  enum Axis
  {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
  };

  void setAxis(int axis);
  int getAxis() { return axis_; }

private:

  float min_value_;
  float max_value_;

  bool auto_compute_bounds_;
  bool use_fixed_frame_;

  int axis_;
//
//  BoolPropertyWPtr auto_compute_bounds_property_;
//  FloatPropertyWPtr min_value_property_;
//  FloatPropertyWPtr max_value_property_;
//  EnumPropertyWPtr axis_property_;
//  BoolPropertyWPtr use_fixed_frame_property_;
};

}

#endif	/* AXIS_COLOR_PC_TRANSFORMER_H */

