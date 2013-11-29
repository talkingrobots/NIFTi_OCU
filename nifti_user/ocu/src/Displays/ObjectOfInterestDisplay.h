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

#ifndef EU_NIFTI_OCU_DISPLAY_EOI_DISPLAY_H
#define EU_NIFTI_OCU_DISPLAY_EOI_DISPLAY_H

#include <map>
#include <set>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>

#include <tf/message_filter.h>

#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

#include "Displays/Markers/ObjectOfInterestMarker.h"

#include "Displays/Display.h"

namespace Ogre
{
    class SceneManager;
    class SceneNode;
}

namespace rviz
{
    class FrameTransformer;
}

namespace eu
{
    namespace nifti
    {
        namespace env
        {
            typedef int16_t UUID;
        }

        namespace ocu
        {
            namespace selection
            {
                class SelectionManager;
            }

            namespace display
            {

                /**
                 * \class ObjectOfInterestDisplay
                 * \brief Displays "markers" for Objects of interest
                 *
                 */
                class ObjectOfInterestDisplay : public rviz::Display
                {
                public:
                    ObjectOfInterestDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
                    virtual ~ObjectOfInterestDisplay();

                    virtual void update(float wall_dt, float ros_dt);

                    virtual void fixedFrameChanged();
                    virtual void reset();
                    
                    void onSelectionManagerDeleted();
                    
                    const ObjectOfInterestMarker* getMarker(eu::nifti::env::UUID uuid) const;

                protected:
                    virtual void onEnable();
                    virtual void onDisable();

                    void subscribe();
                    void unsubscribe();

                    void setMarkerStatus(eu::nifti::env::UUID uuid, eu::nifti::ocu::StatusLevel level, const std::string& text);
                    void deleteMarkerStatus(eu::nifti::env::UUID uuid);
                    
                    // Removes all the markers
                    void clearMarkers();

                    // ROS callback notifying of a new EOI message
                    void onEOIMessageReceived(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg);
                    
                    void failedMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg, tf::FilterFailureReason reason);
                    
                    // Once the message is pop off the queue, it is processed here
                    void processEOIMessage(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg);
                    
                    void addOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg);
                    void modifyOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg);
                    void removeOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr& msg);

                    // Ensures that we don't read and write from the same message queue at the same time
                    boost::mutex queueMutex;
                    
                    // Keeps all markers in a map for quick retrieval
                    std::map<eu::nifti::env::UUID, ObjectOfInterestMarkerPtr> markers;
                    // EOIMessages are added to this queue as they are received, and then processed in the update() function (it was there in RVIZ)
                    std::vector<eu_nifti_env_msg_ros::ElementOfInterestMessage::ConstPtr> messageQueue;

                    Ogre::SceneNode* scene_node_; ///< Scene node all the marker objects are parented to

                    eu::nifti::ocu::selection::SelectionManager* selMgr;

                    message_filters::Subscriber<eu_nifti_env_msg_ros::ElementOfInterestMessage> subscriberEOImessagesFiltered;
                    tf::MessageFilter<eu_nifti_env_msg_ros::ElementOfInterestMessage> tf_filter_;
                  
                    const static std::string ROS_TOPIC;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_DISPLAY_EOI_DISPLAY_H */
