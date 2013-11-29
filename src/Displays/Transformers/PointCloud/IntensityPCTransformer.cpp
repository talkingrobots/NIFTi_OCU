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

#include "Displays/Transformers/PointCloud/IntensityPCTransformer.h"

namespace rviz
{

    uint8_t IntensityPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud) const
    {
        int32_t index = PointCloudTransformer::findChannelIndex(cloud, "intensity");
        if (index == -1)
        {
            index = PointCloudTransformer::findChannelIndex(cloud, "intensities");
        }

        if (index == -1)
        {
            return Support_None;
        }

        return Support_Color;
    }

    uint8_t IntensityPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud) const
    {
        int32_t index = PointCloudTransformer::findChannelIndex(cloud, "intensity");
        if (index == -1)
        {
            index = PointCloudTransformer::findChannelIndex(cloud, "intensities");
        }

        if (index == -1)
        {
            return 0;
        }

        return 255;
    }

    bool IntensityPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
    {
        if (!(mask & Support_Color))
        {
            return false;
        }

        int32_t index = PointCloudTransformer::findChannelIndex(cloud, "intensity");
        if (index == -1)
        {
            index = PointCloudTransformer::findChannelIndex(cloud, "intensities");
        }

        if (index == -1)
        {
            return false;
        }

        const uint32_t offset = cloud->fields[index].offset;
        const uint8_t type = cloud->fields[index].datatype;
        const uint32_t point_step = cloud->point_step;
        const uint32_t num_points = cloud->width * cloud->height;

        float min_intensity = 999999.0f;
        float max_intensity = 0.0f;
        if (auto_compute_intensity_bounds_)
        {
            for (uint32_t i = 0; i < num_points; ++i)
            {
                float val = PointCloudTransformer::valueFromCloud<float>(cloud, offset, type, point_step, i);
                min_intensity = std::min(val, min_intensity);
                max_intensity = std::max(val, max_intensity);
            }

            min_intensity = std::max(0.0f, min_intensity);
            max_intensity = std::min(999999.0f, max_intensity);
            min_intensity_ = min_intensity;
            max_intensity_ = max_intensity;
        } else
        {
            min_intensity = min_intensity_;
            max_intensity = max_intensity_;
        }
        float diff_intensity = max_intensity - min_intensity;
        Ogre::ColourValue max_color = colorMax;
        Ogre::ColourValue min_color = colorMin;

        for (uint32_t i = 0; i < num_points; ++i)
        {
            float val = PointCloudTransformer::valueFromCloud<float>(cloud, offset, type, point_step, i);

            float normalized_intensity = diff_intensity > 0.0f ? (val - min_intensity) / diff_intensity : 1.0f;
            normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
            out.points[i].color.r = max_color.r * normalized_intensity + min_color.r * (1.0f - normalized_intensity);
            out.points[i].color.g = max_color.g * normalized_intensity + min_color.g * (1.0f - normalized_intensity);
            out.points[i].color.b = max_color.b * normalized_intensity + min_color.b * (1.0f - normalized_intensity);
        }

        return true;
    }

    void IntensityPCTransformer::reset()
    {
        min_intensity_ = 0.0f;
        max_intensity_ = 4096.0f;
    }

    const Ogre::ColourValue& IntensityPCTransformer::getMaxColor() const
    {
        return colorMax;
    }

    const Ogre::ColourValue& IntensityPCTransformer::getMinColor() const
    {
        return colorMin;
    }

    void IntensityPCTransformer::setMaxColor(const Ogre::ColourValue& color)
    {
        colorMax = color;

        causeRetransform();
    }

    void IntensityPCTransformer::setMinColor(const Ogre::ColourValue& color)
    {
        colorMin = color;

        causeRetransform();
    }

    void IntensityPCTransformer::setMinIntensity(float val)
    {
        min_intensity_ = val;
        if (min_intensity_ > max_intensity_)
        {
            min_intensity_ = max_intensity_;
        }

        causeRetransform();
    }

    void IntensityPCTransformer::setMaxIntensity(float val)
    {
        max_intensity_ = val;
        if (max_intensity_ < min_intensity_)
        {
            max_intensity_ = min_intensity_;
        }

        causeRetransform();
    }

    void IntensityPCTransformer::setAutoComputeIntensityBounds(bool compute)
    {
        auto_compute_intensity_bounds_ = compute;

        causeRetransform();
    }

}
