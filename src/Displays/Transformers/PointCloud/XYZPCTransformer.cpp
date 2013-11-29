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

#include "XYZPCTransformer.h"

namespace rviz
{

    uint8_t XYZPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud) const
    {
        int32_t xi = PointCloudTransformer::findChannelIndex(cloud, "x");
        int32_t yi = PointCloudTransformer::findChannelIndex(cloud, "y");
        int32_t zi = PointCloudTransformer::findChannelIndex(cloud, "z");

        if (xi == -1 || yi == -1 || zi == -1)
        {
            return Support_None;
        }

        if (cloud->fields[xi].datatype == sensor_msgs::PointField::FLOAT32)
        {
            return Support_XYZ;
        }

        return Support_None;
    }

    bool XYZPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
    {
        if (!(mask & Support_XYZ))
        {
            return false;
        }

        int32_t xi = PointCloudTransformer::findChannelIndex(cloud, "x");
        int32_t yi = PointCloudTransformer::findChannelIndex(cloud, "y");
        int32_t zi = PointCloudTransformer::findChannelIndex(cloud, "z");

        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const uint32_t point_step = cloud->point_step;
        const uint32_t num_points = cloud->width * cloud->height;
        uint8_t const* point = &cloud->data.front();
        for (uint32_t i = 0; i < num_points; ++i, point += point_step)
        {
            float x = *reinterpret_cast<const float*> (point + xoff);
            float y = *reinterpret_cast<const float*> (point + yoff);
            float z = *reinterpret_cast<const float*> (point + zoff);

            Ogre::Vector3 pos(x, y, z);
            pos = transform * pos;
            out.points[i].position = pos;
        }

        return true;
    }

}
