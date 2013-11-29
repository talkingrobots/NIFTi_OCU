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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreWireBoundingBox.h>

#include <ros/time.h>

#include <tf/transform_listener.h>

#include "FloatValidator.h"
#include "NIFTiViewsUtil.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/Transformers/PointCloud/PointCloudTransformer.h"
#include "Displays/Transformers/PointCloud/AxisColorPCTransformer.h"
#include "Displays/Transformers/PointCloud/FlatColorPCTransformer.h"
#include "Displays/Transformers/PointCloud/IntensityPCTransformer.h"
#include "Displays/Transformers/PointCloud/RGB8PCTransformer.h"
#include "Displays/Transformers/PointCloud/RGBF32PCTransformer.h"
#include "Displays/Transformers/PointCloud/XYZPCTransformer.h"

#include "Displays/PointCloudBase.h"


namespace rviz
{

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    PointCloudBase::CloudInfo::CloudInfo()
    : time_(0.0f)
    , transform_(Ogre::Matrix4::ZERO)
    , num_points_(0)
    {
    }

    PointCloudBase::CloudInfo::~CloudInfo()
    {
    }

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    PointCloudBase::PointCloudBase(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameMgr, updateQueue, threadQueue)
    , new_cloud_(false)
    , needsRetransform(false)
    , style_(Billboards)
    , billboard_size_(0.01)
    , point_decay_time_(0.0f)
    , messagesReceived(0)
    , total_point_count_(0)
    {
        cloud_ = new ogre_tools::PointCloud();
        sceneNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
        sceneNode->attachObject(cloud_);

        setStyle(style_);
        setBillboardSize(billboard_size_);
        setAlpha(1.0f);

        // Creates all transformers at once (this may not be the best idea for performance)
        createTransformers();
    }

    PointCloudBase::~PointCloudBase()
    {
        // Deletes all transformers
        delete transformers["XYZ"];
        delete transformers["Intensity"];
        delete transformers["RGB8"];
        delete transformers["RGBF32"];
        delete transformers["Flat Color"];

        sceneMgr->destroySceneNode(sceneNode);

        delete cloud_;
    }

    void PointCloudBase::createTransformers()
    {
        transformers["Axis"] = new AxisColorPCTransformer();
        transformers["XYZ"] = new XYZPCTransformer();
        transformers["Intensity"] = new IntensityPCTransformer();
        transformers["RGB8"] = new RGB8PCTransformer();
        transformers["RGBF32"] = new RGBF32PCTransformer();
        transformers["Flat Color"] = new FlatColorPCTransformer();
    }

    void PointCloudBase::setAlpha(float alpha)
    {
        alpha_ = alpha;

        cloud_->setAlpha(alpha_);
    }

    void PointCloudBase::setDecayTime(float time)
    {
        point_decay_time_ = time;

        causeRender();
    }

    void PointCloudBase::setStyle(int style)
    {
        ROS_ASSERT(style < StyleCount);

        style_ = style;

        ogre_tools::PointCloud::RenderMode mode = ogre_tools::PointCloud::RM_POINTS;
        if (style == Billboards)
        {
            mode = ogre_tools::PointCloud::RM_BILLBOARDS;
        } else if (style == BillboardSpheres)
        {
            mode = ogre_tools::PointCloud::RM_BILLBOARD_SPHERES;
        } else if (style == Boxes)
        {
            mode = ogre_tools::PointCloud::RM_BOXES;
        }

        cloud_->setRenderMode(mode);

        causeRender();
    }

    void PointCloudBase::setBillboardSize(float size)
    {
        billboard_size_ = size;

        cloud_->setDimensions(size, size, size);

        causeRender();
    }

    void PointCloudBase::onEnable()
    {
    }

    void PointCloudBase::onDisable()
    {
        clouds_.clear();
        cloud_->clear();
        messagesReceived = 0;
        total_point_count_ = 0;
    }

    void PointCloudBase::causeRetransform()
    {
        boost::mutex::scoped_lock lock(clouds_mutex_);
        needsRetransform = true;
    }
    
    u_int PointCloudBase::getVisibilityFlags() 
    { 
        return cloud_->getVisibilityFlags();
    }
    
    void PointCloudBase::setVisibilityFlags(u_int flags) 
    { 
        cloud_->setVisibilityFlags(flags); 
    }

    void PointCloudBase::update(float wall_dt, float ros_dt)
    {
        {
            boost::mutex::scoped_lock lock(clouds_mutex_);

            if (needsRetransform)
            {
                retransform();
                needsRetransform = false;
            }

            D_CloudInfo::iterator cloud_it = clouds_.begin();
            D_CloudInfo::iterator cloud_end = clouds_.end();
            for (; cloud_it != cloud_end; ++cloud_it)
            {
                const CloudInfoPtr& info = *cloud_it;

                info->time_ += ros_dt;
            }

            if (point_decay_time_ > 0.0f)
            {
                bool removed = false;
                uint32_t points_to_pop = 0;
                while (!clouds_.empty() && clouds_.front()->time_ > point_decay_time_)
                {
                    total_point_count_ -= clouds_.front()->num_points_;
                    points_to_pop += clouds_.front()->num_points_;
                    clouds_.pop_front();
                    removed = true;
                }

                if (removed)
                {
                    cloud_->popPoints(points_to_pop);
                    causeRender();
                }
            }
        }

        if (new_cloud_)
        {
            boost::mutex::scoped_lock lock(new_clouds_mutex_);

            ogre_tools::PointCloud::setDefaultVisibilityFlags(cloud_->getVisibilityFlags()); // Each new point will take on this value
            
            if (point_decay_time_ == 0.0f)
            {
                clouds_.clear();
                cloud_->clear();

                ROS_ASSERT(!new_points_.empty());
                ROS_ASSERT(!new_clouds_.empty());
                V_Point& points = new_points_.back();
                cloud_->addPoints(&points.front(), points.size());
                clouds_.push_back(new_clouds_.back());

                total_point_count_ = points.size();
            } else
            {
                {
                    VV_Point::iterator it = new_points_.begin();
                    VV_Point::iterator end = new_points_.end();
                    for (; it != end; ++it)
                    {
                        V_Point& points = *it;
                        total_point_count_ += points.size();
                        cloud_->addPoints(&points.front(), points.size());
                    }
                }

                {
                    V_CloudInfo::iterator it = new_clouds_.begin();
                    V_CloudInfo::iterator end = new_clouds_.end();
                    for (; it != end; ++it)
                    {
                        clouds_.push_back(*it);
                    }
                }
            }
            
            ogre_tools::PointCloud::setDefaultVisibilityFlags(0x11111111); // Sets it back to the default value

            new_clouds_.clear();
            new_points_.clear();
            new_cloud_ = false;
        }

        updateStatus();
    }

    void PointCloudBase::updateStatus()
    {
        if (messagesReceived == 0)
        {
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
        } else
        {
            std::stringstream ss;
            ss << messagesReceived << " messages received";
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }

        {
            std::stringstream ss;
            ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
            setStatus("Points", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }
    }

    void PointCloudBase::processMessage(const sensor_msgs::PointCloud2Ptr& cloud)
    {
        CloudInfoPtr info(new CloudInfo());
        info->message_ = cloud;
        info->time_ = 0;

        V_Point points;
        if (transformCloud(info, points, true))
        {
            boost::mutex::scoped_lock lock(new_clouds_mutex_);

            new_clouds_.push_back(info);
            new_points_.push_back(V_Point());
            new_points_.back().swap(points);

            new_cloud_ = true;
        }
    }

    void PointCloudBase::setXYZTransformer(const std::string& name)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        if (xyzTransformerName == name)
        {
            return;
        }

        xyzTransformerName = name;

        causeRetransform();
    }

    void PointCloudBase::setColorTransformer(const std::string& name)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        if (colorTransformerName == name)
        {
            return;
        }

        colorTransformerName = name;

        causeRetransform();
    }

    PointCloudTransformer* PointCloudBase::getXYZTransformer(const sensor_msgs::PointCloud2Ptr& cloud)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

        PointCloudTransformer* t = transformers[xyzTransformerName];
        if (t != NULL && t->supports(cloud) && PointCloudTransformer::Support_XYZ)
        {
            return t;
        } else
        {
            return NULL;
        }
    }

    PointCloudTransformer* PointCloudBase::getColorTransformer(const sensor_msgs::PointCloud2Ptr& cloud)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

        PointCloudTransformer* t = transformers[colorTransformerName];
        if (t != NULL && t->supports(cloud) && PointCloudTransformer::Support_Color)
        {
            return t;
        } else
        {
            return NULL;
        }
    }

    void PointCloudBase::retransform()
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

        cloud_->clear();

        // transformCloud can change the transformers, store them off so we can reset them afterwards
        std::string xyz_trans = xyzTransformerName;
        std::string color_trans = colorTransformerName;

        D_CloudInfo::iterator it = clouds_.begin();
        D_CloudInfo::iterator end = clouds_.end();
        for (; it != end; ++it)
        {
            const CloudInfoPtr& cloud = *it;
            V_Point points;
            transformCloud(cloud, points, false);
            if (!points.empty())
            {
                cloud_->addPoints(&points.front(), points.size());
            }
        }

        xyzTransformerName = xyz_trans;
        colorTransformerName = color_trans;
    }

    bool PointCloudBase::transformCloud(const CloudInfoPtr& info, V_Point& points, bool fully_update_transformers)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

        Ogre::Matrix4 transform = info->transform_;

        if (transform == Ogre::Matrix4::ZERO)
        {
            Ogre::Vector3 pos;
            Ogre::Quaternion orient;
            if (!frameTransformer->getTransform(info->message_->header, pos, orient))
            {
                std::stringstream ss;
                ss << "Failed to transform from frame [" << info->message_->header.frame_id << "] to frame [" << frameTransformer->getFixedFrame() << "]";
                setStatus("Message", eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
                return false;
            }

            transform = Ogre::Matrix4(orient);
            transform.setTrans(pos);
            info->transform_ = transform;
        }

        PointCloud cloud;
        size_t size = info->message_->width * info->message_->height;
        info->num_points_ = size;
        PointCloudPoint default_pt;
        default_pt.color = Ogre::ColourValue(1, 1, 1);
        default_pt.position = Ogre::Vector3::ZERO;
        cloud.points.resize(size, default_pt);

        PointCloudTransformer* xyz_trans = getXYZTransformer(info->message_);
        PointCloudTransformer* color_trans = getColorTransformer(info->message_);

        if (!xyz_trans)
        {
            std::stringstream ss;
            ss << "No position transformer available for cloud";
            setStatus("Message", eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
            return false;
        }

        if (!color_trans)
        {
            std::stringstream ss;
            ss << "No color transformer available for cloud";
            setStatus("Message", eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
            return false;
        }

        xyz_trans->transform(info->message_, PointCloudTransformer::Support_XYZ, transform, cloud);
        color_trans->transform(info->message_, PointCloudTransformer::Support_Color, transform, cloud);

        points.resize(size);
        for (size_t i = 0; i < size; ++i)
        {
            Ogre::Vector3 pos = cloud.points[i].position;
            Ogre::ColourValue color = cloud.points[i].color;
            if (FloatValidator::validateFloats(pos))
            {
                points[i].x = pos.x;
                points[i].y = pos.y;
                points[i].z = pos.z;
            } else
            {
                points[i].x = 999999.0f;
                points[i].y = 999999.0f;
                points[i].z = 999999.0f;
            }
            points[i].setColor(color.r, color.g, color.b);
        }

        return true;
    }

    bool convertPointCloudToPointCloud2(const sensor_msgs::PointCloud& input, sensor_msgs::PointCloud2& output)
    {
        output.header = input.header;
        output.width = input.points.size();
        output.height = 1;
        output.fields.resize(3 + input.channels.size());
        // Convert x/y/z to fields
        output.fields[0].name = "x";
        output.fields[1].name = "y";
        output.fields[2].name = "z";
        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < output.fields.size(); ++d, offset += 4)
        {
            output.fields[d].offset = offset;
            output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        }
        output.point_step = offset;
        output.row_step = output.point_step * output.width;
        // Convert the remaining of the channels to fields
        for (size_t d = 0; d < input.channels.size(); ++d)
            output.fields[3 + d].name = input.channels[d].name;
        output.data.resize(input.points.size() * output.point_step);
        output.is_bigendian = false; // @todo ?
        output.is_dense = false;

        // Copy the data points
        for (size_t cp = 0; cp < input.points.size(); ++cp)
        {
            memcpy(&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof (float));
            memcpy(&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof (float));
            memcpy(&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof (float));
            for (size_t d = 0; d < input.channels.size(); ++d)
                memcpy(&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
        }
        return (true);
    }

    void PointCloudBase::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
    {
        sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
        convertPointCloudToPointCloud2(*cloud, *out);
        addMessage(out);
    }

    void PointCloudBase::addMessage(const sensor_msgs::PointCloud2Ptr& cloud)
    {
        ++messagesReceived;

        if (cloud->width * cloud->height == 0)
        {
            return;
        }

        processMessage(cloud);
    }

    void PointCloudBase::fixedFrameChanged()
    {
        reset();
    }

    void PointCloudBase::onTransformerOptions(V_string& ops, uint32_t mask)
    {
        boost::mutex::scoped_lock clock(clouds_mutex_);

        if (clouds_.empty())
        {
            return;
        }

        boost::recursive_mutex::scoped_lock tlock(transformers_mutex_);

        const sensor_msgs::PointCloud2Ptr& msg = clouds_.front()->message_;

        std::map<std::string, PointCloudTransformer*>::iterator it = transformers.begin();
        std::map<std::string, PointCloudTransformer*>::iterator end = transformers.end();
        for (; it != end; ++it)
        {
            const PointCloudTransformer* t = it->second;
            if ((t->supports(msg) & mask) == mask)
            {
                ops.push_back(it->first);
            }
        }
    }

    void PointCloudBase::reset()
    {
        Display::reset();

        clouds_.clear();
        cloud_->clear();
        messagesReceived = 0;
        total_point_count_ = 0;
    }

} // namespace rviz
