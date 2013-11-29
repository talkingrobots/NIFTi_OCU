// Benoit 2011-05-27

#include <ros/node_handle.h>

#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

#include "Displays/Markers/LocationOfInterestMarker.h"

#include "IVisualizationManager.h"
#include "IMultiVisualizationManager.h"
#include "UUIDsManager.h"
#include "NIFTiROSUtil.h"
#include "NIFTiViewsUtil.h"

#include "Selection/LocationSelectionManager.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {

                const char* LocationSelectionManager::TOPIC_EOI_CREATION = "/eoi";

                LocationSelectionManager::LocationSelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr)
                : multiVizMgr(multiVizMgr)
                , selection(NULL)
                {
                    publisherLOICreation = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<eu_nifti_env_msg_ros::ElementOfInterestMessage > (TOPIC_EOI_CREATION, 1);
                }

                LocationSelectionManager::~LocationSelectionManager()
                {
                    deselect();
                }

                const eu::nifti::ocu::display::LocationOfInterestMarker* LocationSelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y)
                {
                    //ROS_INFO("IN const eu::nifti::ocu::display::LocationOfInterestMarker* LocationOfInterestSelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y)");

                    // For now, handles only the scene views, not the camera views
                    if(!NIFTiViewsUtil::viewTypeIsVirtualScene(vizMgr->getViewType()))
                        return NULL;

                    deselect();

                    // Calculates the real-world position of the click
                    Ogre::Vector3 positionInFixedFrame = vizMgr->getFixedFramePositionFromViewportPosition(x, y);

                    // No intersection found (e.g. the user clicked in the sky)
                    if (positionInFixedFrame == Ogre::Vector3::ZERO)
                    {
                        return NULL;
                    }

                    eu_nifti_env::LocationOfInterest loi;

                    try
                    {
                        loi.element.uuid = eu::nifti::ocu::UUIDsManager::getUUID();
                    }
                    catch (char const*)
                    {
                        // No UUID available
                        std::cerr << "Cannot publish the selection because there is no available UUID" << std::endl;
                        return NULL;
                    }

                    loi.element.sourceType = eu_nifti_env::ElementOfInterest::SOURCE_TYPE_USER;
                    loi.element.type = eu_nifti_env::ElementOfInterest::TYPE_LOCATION;
                    std::stringstream ss;
                    ss << "Location " << loi.element.uuid;
                    loi.element.name = ss.str();

                    loi.point.x = positionInFixedFrame.x;
                    loi.point.y = positionInFixedFrame.y;
                    loi.point.z = positionInFixedFrame.z;

                    selection = new eu::nifti::ocu::display::LocationOfInterestMarker(loi, multiVizMgr->getSceneManager());

                    publishCreationMessage();

                    //ROS_INFO("OUT const eu::nifti::ocu::display::LocationOfInterestMarker* LocationOfInterestSelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y)");

                    return selection;
                }

                void LocationSelectionManager::deselect()
                {
                    delete selection;
                    selection = NULL;
                }

                const eu::nifti::ocu::display::LocationOfInterestMarker* LocationSelectionManager::getSelectedMarker()
                {
                    return selection;
                }

                void LocationSelectionManager::focusOnSelection()
                {
                    if(selection == NULL) return;
                    
                    geometry_msgs::Point point = selection->getLocationOfInterest()->point;
                    Ogre::Vector3 position(point.x, point.y, point.z);

                    multiVizMgr->lookAt(position);
                    // Benoit This might be an annoying behaviour because all views will change at once
                    // Maybe we could make only the "active" view change

                }
                
                void LocationSelectionManager::publishCreationMessage()
                {
                   // Creates a message to tell about the new location
                    eu_nifti_env_msg_ros::ElementOfInterestMessage creationMsg;
                    creationMsg.header.stamp = ros::Time::now();
                    creationMsg.type = eu_nifti_env::ElementOfInterest::TYPE_LOCATION;
                    creationMsg.location = *(selection->getLocationOfInterest());
                    creationMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_ADD;

                    publisherLOICreation.publish(creationMsg);
                    //ROS_INFO("Published a creation message for location #%i", creationMsg.location.element.uuid); 
                }

            }
        }
    }
}