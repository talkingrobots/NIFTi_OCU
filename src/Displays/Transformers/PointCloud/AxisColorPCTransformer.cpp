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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix4.h>

#include "Displays/Transformers/PointCloud/AxisColorPCTransformer.h"

namespace rviz
{

    static void getRainbowColor(float value, Ogre::ColourValue& color)
    {
        value = std::min(value, 1.0f);
        value = std::max(value, 0.0f);

        float h = value * 5.0f + 1.0f;
        int i = floor(h);
        float f = h - i;
        if (!(i & 1)) f = 1 - f; // if i is even
        float n = 1 - f;

        if (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
        else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
        else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
        else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
        else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
    }
    
    uint8_t AxisColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud) const
    {
        return Support_Color;
    }

    uint8_t AxisColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        return 255;
    }

    bool AxisColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
    {
        if (!(mask & Support_Color))
        {
            return false;
        }

        int32_t xi = findChannelIndex(cloud, "x");
        int32_t yi = findChannelIndex(cloud, "y");
        int32_t zi = findChannelIndex(cloud, "z");

        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const uint32_t point_step = cloud->point_step;
        const uint32_t num_points = cloud->width * cloud->height;
        uint8_t const* point = &cloud->data.front();

        // compute bounds

        float min_value_current = 9999.0f;
        float max_value_current = -9999.0f;
        std::vector<float> values;
        values.reserve(num_points);

        for (uint32_t i = 0; i < num_points; ++i, point += point_step)
        {
            float x = *reinterpret_cast<const float*> (point + xoff);
            float y = *reinterpret_cast<const float*> (point + yoff);
            float z = *reinterpret_cast<const float*> (point + zoff);

            Ogre::Vector3 pos(x, y, z);

            if (use_fixed_frame_)
            {
                pos = transform * pos;
            }

            float val = pos[axis_];
            min_value_current = std::min(min_value_current, val);
            max_value_current = std::max(max_value_current, val);

            values.push_back(val);
        }

        if (auto_compute_bounds_)
        {
            min_value_ = min_value_current;
            max_value_ = max_value_current;
        }

        for (uint32_t i = 0; i < num_points; ++i)
        {
            float range = std::max(max_value_ - min_value_, 0.001f);
            float value = 1.0 - (values[i] - min_value_) / range;
            getRainbowColor(value, out.points[i].color);
        }

        return true;
    }

//    void AxisColorPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props)
//    {
//        if (mask & Support_Color)
//        {
//            axis_property_ = property_man->createProperty<EnumProperty > ("Axis", prefix, boost::bind(&AxisColorPCTransformer::getAxis, this), boost::bind(&AxisColorPCTransformer::setAxis, this, _1),
//                    parent, this);
//            EnumPropertyPtr prop = axis_property_.lock();
//            prop->addOption("X", AXIS_X);
//            prop->addOption("Y", AXIS_Y);
//            prop->addOption("Z", AXIS_Z);
//            setPropertyHelpText(axis_property_, "The axis to interpolate the color along.");
//            auto_compute_bounds_property_ = property_man->createProperty<BoolProperty > ("Autocompute Value Bounds", prefix, boost::bind(&AxisColorPCTransformer::getAutoComputeBounds, this),
//                    boost::bind(&AxisColorPCTransformer::setAutoComputeBounds, this, _1), parent, this);
//
//            setPropertyHelpText(auto_compute_bounds_property_, "Whether to automatically compute the value min/max values.");
//            min_value_property_ = property_man->createProperty<FloatProperty > ("Min Value", prefix, boost::bind(&AxisColorPCTransformer::getMinValue, this),
//                    boost::bind(&AxisColorPCTransformer::setMinValue, this, _1), parent, this);
//            setPropertyHelpText(min_value_property_, "Minimum value value, used to interpolate the color of a point.");
//            max_value_property_ = property_man->createProperty<FloatProperty > ("Max Value", prefix, boost::bind(&AxisColorPCTransformer::getMaxValue, this),
//                    boost::bind(&AxisColorPCTransformer::setMaxValue, this, _1), parent, this);
//            setPropertyHelpText(max_value_property_, "Maximum value value, used to interpolate the color of a point.");
//
//            use_fixed_frame_property_ = property_man->createProperty<BoolProperty > ("Use Fixed Frame", prefix, boost::bind(&AxisColorPCTransformer::getUseFixedFrame, this),
//                    boost::bind(&AxisColorPCTransformer::setUseFixedFrame, this, _1), parent, this);
//            setPropertyHelpText(use_fixed_frame_property_, "Whether to color the cloud based on its fixed frame position or its local frame position.");
//
//            out_props.push_back(axis_property_);
//            out_props.push_back(auto_compute_bounds_property_);
//            out_props.push_back(min_value_property_);
//            out_props.push_back(max_value_property_);
//            out_props.push_back(use_fixed_frame_property_);
//
//            if (auto_compute_bounds_)
//            {
//                hideProperty(min_value_property_);
//                hideProperty(max_value_property_);
//            }
//            else
//            {
//                showProperty(min_value_property_);
//                showProperty(max_value_property_);
//            }
//        }
//    }

    void AxisColorPCTransformer::setUseFixedFrame(bool use)
    {
        use_fixed_frame_ = use;
//        propertyChanged(use_fixed_frame_property_);
        causeRetransform();
    }

    void AxisColorPCTransformer::setAxis(int axis)
    {
        axis_ = axis;
//        propertyChanged(axis_property_);
        causeRetransform();
    }

    void AxisColorPCTransformer::setMinValue(float val)
    {
        min_value_ = val;
        if (min_value_ > max_value_)
        {
            min_value_ = max_value_;
        }

//        propertyChanged(min_value_property_);

        causeRetransform();
    }

    void AxisColorPCTransformer::setMaxValue(float val)
    {
        max_value_ = val;
        if (max_value_ < min_value_)
        {
            max_value_ = min_value_;
        }

//        propertyChanged(max_value_property_);

        causeRetransform();
    }

    void AxisColorPCTransformer::setAutoComputeBounds(bool compute)
    {
        auto_compute_bounds_ = compute;

//        if (auto_compute_bounds_)
//        {
//            hideProperty(min_value_property_);
//            hideProperty(max_value_property_);
//        }
//        else
//        {
//            showProperty(min_value_property_);
//            showProperty(max_value_property_);
//        }
//
//        propertyChanged(auto_compute_bounds_property_);

        causeRetransform();
    }


}
