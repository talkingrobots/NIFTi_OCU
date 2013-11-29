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

#include <ogre_tools/arrow.h>
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>

#include <tf/transform_listener.h>

#include "FloatValidator.h"

#include "Displays/Markers/marker_base.h"
#include "Displays/Markers/GenericMarkerCreator.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Selection/Common.h"

#include "Displays/MarkerDisplay.h"

namespace rviz
{

    MarkerDisplay::MarkerDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, GenericMarkerCreator* markerCreator, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
    , markerCreator(markerCreator)
    , tf_filter_(*frameTransformer->getTFClient(), "", 100, update_nh_)
    {
        assert(markerCreator != NULL);

        this->topic = topic;

        scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

        tf_filter_.connectInput(sub_);
        tf_filter_.registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));
        tf_filter_.registerFailureCallback(boost::bind(&MarkerDisplay::failedMarker, this, _1, _2));
    }

    MarkerDisplay::~MarkerDisplay()
    {
        unsubscribe();

        clearMarkers();
    }

    MarkerBasePtr MarkerDisplay::getMarker(MarkerID id)
    {
        std::map<MarkerID, MarkerBasePtr>::iterator it = markers_.find(id);
        if (it != markers_.end())
        {
            return it->second;
        }

        return MarkerBasePtr();
    }

    void MarkerDisplay::clearMarkers()
    {
        markers_.clear();
        markers_with_expiration_.clear();
        frame_locked_markers_.clear();
        tf_filter_.clear();
    }

    void MarkerDisplay::onEnable()
    {
        subscribe();

        scene_node_->setVisible(true);
    }

    void MarkerDisplay::onDisable()
    {
        unsubscribe();
        tf_filter_.clear();

        clearMarkers();

        scene_node_->setVisible(false);
    }

    void MarkerDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        array_sub_.shutdown();
        sub_.unsubscribe();

        try
        {
            sub_.subscribe(update_nh_, topic, 1000);
            array_sub_ = update_nh_.subscribe(topic + "_array", 1000, &MarkerDisplay::incomingMarkerArray, this);
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, "OK");
        }
        catch (ros::Exception& e)
        {
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, std::string("Error subscribing: ") + e.what());
        }
    }

    void MarkerDisplay::unsubscribe()
    {
        sub_.unsubscribe();
        array_sub_.shutdown();
    }

    void MarkerDisplay::deleteMarker(MarkerID id)
    {
        deleteMarkerStatus(id);

        std::map<MarkerID, MarkerBasePtr>::iterator it = markers_.find(id);
        if (it != markers_.end())
        {
            markers_with_expiration_.erase(it->second);
            frame_locked_markers_.erase(it->second);
            markers_.erase(it);
        }
    }

    void MarkerDisplay::setNamespaceEnabled(const std::string& ns, bool enabled)
    {
        std::map<std::string, Namespace>::iterator it = namespaces_.find(ns);
        if (it != namespaces_.end())
        {
            it->second.enabled = enabled;

            std::vector<MarkerID> to_delete;

            // TODO: this is inefficient, should store every in-use id per namespace and lookup by that
            std::map<MarkerID, MarkerBasePtr>::iterator marker_it = markers_.begin();
            std::map<MarkerID, MarkerBasePtr>::iterator marker_end = markers_.end();
            for (; marker_it != marker_end; ++marker_it)
            {
                if (marker_it->first.first == ns)
                {
                    to_delete.push_back(marker_it->first);
                }
            }

            {
                std::vector<MarkerID>::iterator it = to_delete.begin();
                std::vector<MarkerID>::iterator end = to_delete.end();
                for (; it != end; ++it)
                {
                    deleteMarker(*it);
                }
            }
        }
    }

    bool MarkerDisplay::isNamespaceEnabled(const std::string& ns)
    {
        std::map<std::string, Namespace>::iterator it = namespaces_.find(ns);
        if (it != namespaces_.end())
        {
            return it->second.enabled;
        }

        return true;
    }

    void MarkerDisplay::setMarkerStatus(MarkerID id, eu::nifti::ocu::StatusLevel level, const std::string& text)
    {
        std::stringstream ss;
        ss << id.first << "/" << id.second;
        std::string marker_name = ss.str();
        setStatus(marker_name, level, text);
    }

    void MarkerDisplay::deleteMarkerStatus(MarkerID id)
    {
        std::stringstream ss;
        ss << id.first << "/" << id.second;
        std::string marker_name = ss.str();
        deleteStatus(marker_name);
    }

    void MarkerDisplay::incomingMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array)
    {
        std::vector<visualization_msgs::Marker>::const_iterator it = array->markers.begin();
        std::vector<visualization_msgs::Marker>::const_iterator end = array->markers.end();
        for (; it != end; ++it)
        {
            const visualization_msgs::Marker& marker = *it;
            tf_filter_.add(visualization_msgs::Marker::Ptr(new visualization_msgs::Marker(marker)));
        }
    }

    void MarkerDisplay::incomingMarker(const visualization_msgs::Marker::ConstPtr& marker)
    {
        boost::mutex::scoped_lock lock(queue_mutex_);

        message_queue_.push_back(marker);
    }

    void MarkerDisplay::failedMarker(const visualization_msgs::Marker::ConstPtr& marker, tf::FilterFailureReason reason)
    {
        std::string error = frameTransformer->discoverFailureReason(marker->header.frame_id, marker->header.stamp, marker->__connection_header ? (*marker->__connection_header)["callerid"] : "unknown", reason);
        setMarkerStatus(MarkerID(marker->ns, marker->id), eu::nifti::ocu::STATUS_LEVEL_ERROR, error);
    }

    void MarkerDisplay::processMessage(const visualization_msgs::Marker::ConstPtr& message)
    {
        if (!FloatValidator::validateFloats(*message))
        {
            setMarkerStatus(MarkerID(message->ns, message->id), eu::nifti::ocu::STATUS_LEVEL_ERROR, "Contains invalid floating point values (nans or infs)");
            return;
        }

        switch (message->action)
        {
            case visualization_msgs::Marker::ADD:
                processAdd(message);
                break;

            case visualization_msgs::Marker::DELETE:
                processDelete(message);
                break;

            default:
                ROS_ERROR("Unknown marker action: %d\n", message->action);
        }
    }

    void MarkerDisplay::processAdd(const visualization_msgs::Marker::ConstPtr& message)
    {
        std::map<std::string, Namespace>::iterator ns_it = namespaces_.find(message->ns);
        if (ns_it == namespaces_.end())
        {
            Namespace ns;
            ns.name = message->ns;
            ns.enabled = true;

            ns_it = namespaces_.insert(std::make_pair(ns.name, ns)).first;
        }

        if (!ns_it->second.enabled)
        {
            return;
        }

        deleteMarkerStatus(MarkerID(message->ns, message->id));

        bool create = true;
        MarkerBasePtr marker;

        std::map<MarkerID, MarkerBasePtr>::iterator it = markers_.find(MarkerID(message->ns, message->id));
        if (it != markers_.end())
        {
            marker = it->second;
            markers_with_expiration_.erase(marker);
            if (message->type == marker->getMessage()->type)
            {
                create = false;
            } else
            {
                markers_.erase(it);
            }
        }

        if (create)
        {
            marker.reset(markerCreator->createMarker(message, this, sceneMgr, frameTransformer, scene_node_));
            
            markers_.insert(std::make_pair(MarkerID(message->ns, message->id), marker));
        }

        if (marker)
        {
            marker->setMessage(message);

            if (message->lifetime.toSec() > 0.0001f)
            {
                markers_with_expiration_.insert(marker);
            }

            if (message->frame_locked)
            {
                frame_locked_markers_.insert(marker);
            }

            causeRender();
        }
    }

    void MarkerDisplay::processDelete(const visualization_msgs::Marker::ConstPtr& message)
    {
        deleteMarker(MarkerID(message->ns, message->id));
        causeRender();
    }

    void MarkerDisplay::update(float wall_dt, float ros_dt)
    {
        std::vector<visualization_msgs::Marker::ConstPtr> local_queue;

        {
            boost::mutex::scoped_lock lock(queue_mutex_);

            local_queue.swap(message_queue_);
        }

        if (!local_queue.empty())
        {
            std::vector<visualization_msgs::Marker::ConstPtr>::iterator message_it = local_queue.begin();
            std::vector<visualization_msgs::Marker::ConstPtr>::iterator message_end = local_queue.end();
            for (; message_it != message_end; ++message_it)
            {
                visualization_msgs::Marker::ConstPtr& marker = *message_it;

                processMessage(marker);
            }
        }

        {
            std::set<MarkerBasePtr>::iterator it = markers_with_expiration_.begin();
            std::set<MarkerBasePtr>::iterator end = markers_with_expiration_.end();
            for (; it != end;)
            {
                MarkerBasePtr marker = *it;
                if (marker->expired())
                {
                    std::set<MarkerBasePtr>::iterator copy = it;
                    ++it;
                    deleteMarker(marker->getID());
                } else
                {
                    ++it;
                }
            }
        }

        {
            std::set<MarkerBasePtr>::iterator it = frame_locked_markers_.begin();
            std::set<MarkerBasePtr>::iterator end = frame_locked_markers_.end();
            for (; it != end; ++it)
            {
                MarkerBasePtr marker = *it;
                marker->updateFrameLocked();
            }
        }
    }

    void MarkerDisplay::fixedFrameChanged()
    {
        tf_filter_.setTargetFrame(fixed_frame_);

        clearMarkers();
    }

    void MarkerDisplay::reset()
    {
        Display::reset();
        clearMarkers();
    }

} // namespace rviz
