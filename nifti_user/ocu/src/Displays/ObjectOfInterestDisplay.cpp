// Benoit 2011-05-24 Comes from RVIZ Marker Display

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

#include <tf/transform_listener.h>

#include <eu_nifti_env_msg_ros/Util.h>

//#include "FloatValidator.h"

#include "Displays/Markers/OOIMarkerFactory.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Selection/SelectionManager.h"

#include "Displays/ObjectOfInterestDisplay.h"

using namespace eu_nifti_env;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                const std::string ObjectOfInterestDisplay::ROS_TOPIC = "/eoi/draw";

                ObjectOfInterestDisplay::ObjectOfInterestDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
                : rviz::Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
                , selMgr(selMgr)
                , tf_filter_(*frameTransformer->getTFClient(), "", 100, update_nh_)
                {
                    this->topic = ROS_TOPIC;

                    scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

                    tf_filter_.connectInput(subscriberEOImessagesFiltered);
                    tf_filter_.registerCallback(boost::bind(&ObjectOfInterestDisplay::onEOIMessageReceived, this, _1));
                    tf_filter_.registerFailureCallback(boost::bind(&ObjectOfInterestDisplay::failedMarker, this, _1, _2));

                    OOIMarkerFactory::initialize(sceneMgr, frameTransformer, scene_node_);
                }

                ObjectOfInterestDisplay::~ObjectOfInterestDisplay()
                {
                    unsubscribe();

                    clearMarkers();
                }

                void ObjectOfInterestDisplay::onSelectionManagerDeleted()
                {
                    selMgr = NULL;
                }

                const ObjectOfInterestMarker* ObjectOfInterestDisplay::getMarker(eu::nifti::env::UUID uuid) const
                {
                    std::map<eu::nifti::env::UUID, ObjectOfInterestMarkerPtr>::const_iterator it = markers.find(uuid);
                    if (it != markers.end())
                    {
                        return it->second.get();
                    }

                    return NULL;
                }

                void ObjectOfInterestDisplay::clearMarkers()
                {

                    // Goes through all markers and notifies the Selection Manager not to follow these objects anymore
                    if (selMgr != NULL)
                    {
                        std::map<eu::nifti::env::UUID, ObjectOfInterestMarkerPtr>::const_iterator it = markers.begin();
                        while (it != markers.end())
                        {
                            selMgr->removeTrackedMarker(it->second->getCollisionObjectHandle());
                        }
                    }

                    markers.clear();
                    tf_filter_.clear();
                    // Todo: Also remove marker statuses
                }

                void ObjectOfInterestDisplay::onEnable()
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::onEnable()" << std::endl;

                    // RVIZ used to un/subscribe, but now it's on all the time, so it subscribes only the first time
                    if(subscriberEOImessagesFiltered.getTopic() == "")
                    {
                        subscribe();
                    }

                    scene_node_->setVisible(true);

                    //std::cout << "OUT void ObjectOfInterestDisplay::onEnable()" << std::endl;
                }

                void ObjectOfInterestDisplay::onDisable()
                {
                    // RVIZ used to unsubscribe, but I don't want to to stay up to date
                    //unsubscribe();
                    //tf_filter_.clear();

                    // RVIZ used to clear the markers, but I don't want to
                    //clearMarkers();

                    scene_node_->setVisible(false);
                }

                void ObjectOfInterestDisplay::subscribe()
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::subscribe()" << std::endl;

                    // That was from RVIZ, but I want to stay subscribed at all times
                    //if (!isEnabled())
                    //{
                    //    return;
                    //}

                    // That was from RVIZ
                    //sub_.unsubscribe();

                    try
                    {
                        subscriberEOImessagesFiltered.subscribe(update_nh_, ROS_TOPIC, 100);
                        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, "OK");
                    }
                    catch (ros::Exception& e)
                    {
                        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, std::string("Error subscribing: ") + e.what());
                    }

                    //std::cout << "OUT void ObjectOfInterestDisplay::subscribe()" << std::endl;
                }

                void ObjectOfInterestDisplay::unsubscribe()
                {
                    subscriberEOImessagesFiltered.unsubscribe();
                }

                void ObjectOfInterestDisplay::setMarkerStatus(eu::nifti::env::UUID uuid, eu::nifti::ocu::StatusLevel level, const std::string& text)
                {
                    std::stringstream ss;
                    ss << "Marker UUID#" << uuid;
                    setStatus(ss.str(), level, text);
                }

                void ObjectOfInterestDisplay::deleteMarkerStatus(eu::nifti::env::UUID uuid)
                {
                    std::stringstream ss;
                    ss << "Marker UUID#" << uuid;
                    deleteStatus(ss.str());
                }

                void ObjectOfInterestDisplay::failedMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg, tf::FilterFailureReason reason)
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::failedMarker" << std::endl;

                    std::string error = frameTransformer->discoverFailureReason(msg->header.frame_id, msg->header.stamp, msg->__connection_header ? (*msg->__connection_header)["callerid"] : "unknown", reason);
                    setMarkerStatus(eu::nifti::env::msg::ros::Util::getUUID(msg), eu::nifti::ocu::STATUS_LEVEL_ERROR, error);

                    //std::cout << "OUT void ObjectOfInterestDisplay::failedMarker" << std::endl;
                }

                void ObjectOfInterestDisplay::onEOIMessageReceived(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg)
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::onEOIMessageReceived at ROS time " << msg->header.stamp << std::endl;

                    boost::mutex::scoped_lock lock(queueMutex);

                    messageQueue.push_back(msg);

                    //std::cout << "OUT void ObjectOfInterestDisplay::onEOIMessageReceived at ROS time " << msg->header.stamp << std::endl;
                }

                void ObjectOfInterestDisplay::processEOIMessage(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg)
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::processEOIMessage at ROS time " << msg->header.stamp << std::endl;

                    // Todo Implement a systematic verification of all float values in the message
                    //if (!rviz::FloatValidator::validateFloats(*message))
                    //{
                    //    setMarkerStatus(MarkerID(message->ns, message->id), eu::nifti::ocu::STATUS_LEVEL_ERROR, "Contains invalid floating point values (nans or infs)");
                    //    return;
                    //}

                    eu::nifti::env::UUID uuid = eu::nifti::env::msg::ros::Util::getUUID(msg);
                    bool found = (markers.find(uuid) != markers.end());

                    // Dispatches the message to the appropriate function
                    switch (msg->action)
                    {
                        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_ADD:

                            if (found == true)
                            {
                                ROS_WARN("ADD_MARKER message received, but marker already existed. UUID #%d", uuid);
                                return;
                            }
                            addOOIMarker(msg);

                            break;

                        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_MODIFY:

                            if (found == false)
                            {
                                ROS_WARN("MODIFY_MARKER message received, but marker not found. UUID #%d", uuid);
                                addOOIMarker(msg);
                            }
                            else
                            {
                                modifyOOIMarker(msg);
                            }
                            break;

                        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_REMOVE:

                            if (found == false)
                            {
                                ROS_WARN("DELETE_MARKER message received, but marker not found. UUID #%d", uuid);
                                return;
                            }

                            removeOOIMarker(msg);
                            break;
                        default:
                            ROS_ERROR("Unknown ElementOfInterestMessage action: %d", msg->action);
                    }

                    causeRender();

                    //std::cout << "OUT void ObjectOfInterestDisplay::processEOIMessage at ROS time " << msg->header.stamp << std::endl;
                }

                void ObjectOfInterestDisplay::addOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg)
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::addMarker at ROS time " << msg->header.stamp << std::endl;

                    ObjectOfInterestMarkerPtr marker;
                    try
                    {
                        marker = OOIMarkerFactory::getOOIMarker(msg);
                    }
                    catch (char const* s)
                    {
                        // No UUID available
                        ROS_WARN("Ignoring EOI message because: %s", s);
                        return;
                    }

                    // Stores the marker and ensures that it did not already exist
                    std::pair < std::map<eu::nifti::env::UUID, ObjectOfInterestMarkerPtr>::iterator, bool> alreadyExistsCheck = markers.insert(std::make_pair(marker->getObjectOfInterest()->element.uuid, marker));
                    assert(alreadyExistsCheck.second == true);

                    marker->addToOgre();
                    
                    // Tells the Selection Manager to keep track of this object
                    assert(selMgr != NULL);
                    rviz::CollObjectHandle handle = selMgr->addTrackedMarker(marker.get());
                    marker->setCollisionObjectHandle(handle);

                    try
                    {
                        marker->modifyInOgre(msg->header.frame_id, msg->header.stamp, marker->getObjectOfInterest()->pose);
                    }
                    catch (std::string& s) // Means that the transform went wrong
                    {
                        setMarkerStatus(marker->getObjectOfInterest()->element.uuid, eu::nifti::ocu::STATUS_LEVEL_ERROR, s);
                        ROS_WARN("Problem with marker uuid#%d: %s", marker->getObjectOfInterest()->element.uuid, s.c_str());
                    }


                    //std::cout << "OUT void ObjectOfInterestDisplay::addMarker at ROS time " << msg->header.stamp << std::endl;
                }

                void ObjectOfInterestDisplay::modifyOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg)
                {
                    //std::cout << "IN void ObjectOfInterestDisplay::modifyMarker at ROS time " << msg->header.stamp << std::endl;

                    eu::nifti::env::UUID uuid = eu::nifti::env::msg::ros::Util::getUUID(msg);

                    // Find marker and ensures that it's there
                    std::map< eu::nifti::env::UUID, ObjectOfInterestMarkerPtr >::const_iterator result = markers.find(uuid);
                    assert(result != markers.end());

                    ObjectOfInterestMarker* marker = result->second.get(); // Do not delete, because it points to the internal structure of an object

                    // Saves the new position from the message into the marker
                    marker->saveOOI(msg);

                    try
                    {
                        marker->modifyInOgre(msg->header.frame_id, msg->header.stamp, marker->getObjectOfInterest()->pose);
                    }
                    catch (std::string& s) // Means that the transform went wrong
                    {
                        setMarkerStatus(marker->getObjectOfInterest()->element.uuid, eu::nifti::ocu::STATUS_LEVEL_ERROR, s);
                        ROS_WARN("Problem with marker uuid#%d: %s", marker->getObjectOfInterest()->element.uuid, s.c_str());
                    }

                    //std::cout << "OUT void ObjectOfInterestDisplay::modifyMarker at ROS time " << msg->header.stamp << std::endl;
                }

                void ObjectOfInterestDisplay::removeOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg)
                {
                    eu::nifti::env::UUID uuid = eu::nifti::env::msg::ros::Util::getUUID(msg);

                    deleteMarkerStatus(uuid);

                    // Tells the selection manager not to keep track of this object anymore
                    if (selMgr != NULL)
                    {
                        ObjectOfInterestMarkerPtr marker = markers.find(uuid)->second;
                        selMgr->removeTrackedMarker(marker->getCollisionObjectHandle());
                    }

                    int numRemoved = markers.erase(uuid);
                    assert(numRemoved == 1); // Ensures that the marker was really removed
                }

                void ObjectOfInterestDisplay::update(float wall_dt, float ros_dt)
                {
                    std::vector<eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr> localQueue;

                    // Swaps the main queue with a local, to process the messages while still being able to receive some
                    {
                        boost::mutex::scoped_lock lock(queueMutex);

                        localQueue.swap(messageQueue);
                    }

                    // If messages have been received since the last update
                    if (!localQueue.empty())
                    {
                        std::vector<eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr>::iterator message_it = localQueue.begin();
                        std::vector<eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr>::iterator message_end = localQueue.end();
                        for (; message_it != message_end; ++message_it)
                        {
                            processEOIMessage(*message_it);
                        }
                    }
                }

                void ObjectOfInterestDisplay::fixedFrameChanged()
                {
                    tf_filter_.setTargetFrame(fixed_frame_);
                }

                void ObjectOfInterestDisplay::reset()
                {
                    rviz::Display::reset();
                    clearMarkers();
                }

            }
        }
    }
}
