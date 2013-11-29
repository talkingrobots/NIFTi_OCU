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

#ifndef INTENSITY_PC_TRANSFORMER_H
#define	INTENSITY_PC_TRANSFORMER_H

#include <OGRE/OgreColourValue.h>

#include "Displays/Transformers/PointCloud/PointCloudTransformer.h"

namespace rviz
{

    class IntensityPCTransformer : public PointCloudTransformer
    {
    public:

        IntensityPCTransformer()
        : colorMin(0.0f, 0.0f, 0.0f, 1.0f)
        , colorMax(1.0f, 1.0f, 1.0f, 1.0f)
        , min_intensity_(0.0f)
        , max_intensity_(4096.0f)
        {
        }

        virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) const;
        virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
        virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) const;
        virtual void reset();

        void setMinColor(const Ogre::ColourValue& color);
        void setMaxColor(const Ogre::ColourValue& color);

        const Ogre::ColourValue& getMaxColor() const;
        const Ogre::ColourValue& getMinColor() const;

        void setMinIntensity(float val);
        void setMaxIntensity(float val);

        float getMinIntensity() const
        {
            return min_intensity_;
        }

        float getMaxIntensity() const
        {
            return max_intensity_;
        }
        void setAutoComputeIntensityBounds(bool compute);

        bool getAutoComputeIntensityBounds() const
        {
            return auto_compute_intensity_bounds_;
        }

    private:
        Ogre::ColourValue colorMin;
        Ogre::ColourValue colorMax;
        float min_intensity_;
        float max_intensity_;
        bool auto_compute_intensity_bounds_;
        bool intensity_bounds_changed_;

        RetransformFunc retransform_func_;
    };

}

#endif	/* INTENSITY_PC_TRANSFORMER_H */

