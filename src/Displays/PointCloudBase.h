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

#ifndef RVIZ_POINT_CLOUD_BASE_H
#define RVIZ_POINT_CLOUD_BASE_H

#include <deque>
#include <map>
#include <queue>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/signals/connection.hpp>
#include <boost/signals/trackable.hpp>

#include "ogre_tools/point_cloud.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

#include "Displays/Display.h"

namespace rviz
{
    typedef std::vector<std::string> V_string;

    class PointCloudTransformer;

    /**
     * \class PointCloudBase
     * \brief Displays a point cloud of type sensor_msgs::PointCloud
     *
     * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
     * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
     * all being 8 bits.
     */
    class PointCloudBase : public Display, public boost::signals::trackable
    {
    private:

        struct CloudInfo
        {
            CloudInfo();
            ~CloudInfo();

            float time_;

            Ogre::Matrix4 transform_;
            sensor_msgs::PointCloud2Ptr message_;
            uint32_t num_points_;
        };
        typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;
        typedef std::deque<CloudInfoPtr> D_CloudInfo;
        typedef std::vector<CloudInfoPtr> V_CloudInfo;
        typedef std::queue<CloudInfoPtr> Q_CloudInfo;

    public:

        /**
         * \enum Style
         * \brief The different styles of pointcloud drawing
         */
        enum Style
        {
            Points, ///< Points -- points are drawn as a fixed size in 2d space, ie. always 1 pixel on screen
            Billboards, ///< Billboards -- points are drawn as camera-facing quads in 3d space
            BillboardSpheres, ///< Billboard "spheres" -- cam-facing tris with a pixel shader that causes them to look like spheres
            Boxes, ///< Boxes -- Actual 3d cube geometry

            StyleCount,
        };

        /**
         * \enum ChannelRender
         * \brief The different channels that we support rendering
         */
        enum ChannelRender
        {
            Intensity, ///< Intensity data
            Curvature, ///< Surface curvature estimates
            ColorRGBSpace, ///< RGB Color
            NormalSphere, ///< Use the nx-ny-nz (normal coordinates) instead of x-y-z

            ChannelRenderCount,
        };

        PointCloudBase(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        ~PointCloudBase();
        /**
         * \brief Set the rendering style
         * @param style The rendering style
         */
        void setStyle(int style);
        /**
         * \brief Sets the size each point will be when drawn in 3D as a billboard
         * @note Only applicable if the style is set to Billboards (default)
         * @param size The size
         */
        void setBillboardSize(float size);
        /**
         * \brief Set the amount of time each cloud should stick around for
         * @param time Decay time, in seconds
         */
        void setDecayTime(float time);

        float getBillboardSize()
        {
            return billboard_size_;
        }

        int getStyle()
        {
            return style_;
        }

        float getDecayTime()
        {
            return point_decay_time_;
        }

        float getAlpha()
        {
            return alpha_;
        }
        void setAlpha(float alpha);
        
        // Benoit: I changed the following 4 getters and setters from protected to public
        void setXYZTransformer(const std::string& name);
        void setColorTransformer(const std::string& name);

        const std::string& getXYZTransformer()
        {
            return xyzTransformerName;
        }

        const std::string& getColorTransformer()
        {
            return colorTransformerName;
        }

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void reset();
        virtual void update(float wall_dt, float ros_dt);

        void causeRetransform();
        
        // Gets the visiblity flags from the cloud OGRE object.
        u_int getVisibilityFlags();
        
        // Sets the visiblity flags for the cloud OGRE object.
        void setVisibilityFlags(u_int flags);

    protected:
        virtual void onEnable();
        virtual void onDisable();

        typedef std::vector<ogre_tools::PointCloud::Point> V_Point;
        typedef std::vector<V_Point> VV_Point;

        /**
         * \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
         */
        bool transformCloud(const CloudInfoPtr& cloud, V_Point& points, bool fully_update_transformers);

        void processMessage(const sensor_msgs::PointCloud2Ptr& cloud);
        void addMessage(const sensor_msgs::PointCloudConstPtr& cloud);
        void addMessage(const sensor_msgs::PointCloud2Ptr& cloud);
        void updateStatus();

        
        PointCloudTransformer* getXYZTransformer(const sensor_msgs::PointCloud2Ptr& cloud);
        PointCloudTransformer* getColorTransformer(const sensor_msgs::PointCloud2Ptr& cloud);
        void retransform();
        void onTransformerOptions(V_string& ops, uint32_t mask);

        void createTransformers();

        std::map<std::string, PointCloudTransformer*> transformers;

        D_CloudInfo clouds_;
        boost::mutex clouds_mutex_;
        bool new_cloud_;

        ogre_tools::PointCloud* cloud_;

        Ogre::SceneNode* sceneNode;

        VV_Point new_points_;
        V_CloudInfo new_clouds_;
        boost::mutex new_clouds_mutex_;

        float alpha_;

        boost::recursive_mutex transformers_mutex_;
        std::string xyzTransformerName;
        std::string colorTransformerName;
        bool needsRetransform;

        int style_; ///< Our rendering style
        float billboard_size_; ///< Size to draw our billboards
        float point_decay_time_; ///< How long clouds should stick around for before they are culled

        uint32_t messagesReceived;
        uint32_t total_point_count_;
    };

} // namespace rviz

#endif // RVIZ_POINT_CLOUD_BASE_H
