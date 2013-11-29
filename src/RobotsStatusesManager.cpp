// Benoit 2011-09-01

#include <ros/node_handle.h>

#include "IRobotStatusListener.h"
#include "NIFTiROSUtil.h"

#include "RobotsStatusesManager.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const char* RobotsStatusesManager::TOPIC_FLIPPERS_CURRENTS = "/currents";
            //            const char* RobotsStatusesManager::TOPIC_FLIPPERS_POSITIONS = "/flippers_state";
            const char* RobotsStatusesManager::TOPIC_STATUS = "/robot_status";

            std::set<IRobotStatusListener*> RobotsStatusesManager::listeners;

            ros::Subscriber RobotsStatusesManager::subscriberFlippersCurrents;
            //            ros::Subscriber RobotsStatusesManager::subscriberFlippersPositions;
            ros::Subscriber RobotsStatusesManager::subscriberStatus;

            nifti_robot_driver_msgs::CurrentsStampedConstPtr RobotsStatusesManager::flippersCurrents;
            //            nifti_robot_driver_msgs::FlippersStateConstPtr RobotsStatusesManager::flippersPositions;
            nifti_robot_driver_msgs::RobotStatusStampedConstPtr RobotsStatusesManager::status;

            void RobotsStatusesManager::init()
            {
                subscriberFlippersCurrents = NIFTiROSUtil::getNodeHandle()->subscribe(TOPIC_FLIPPERS_CURRENTS, 1, &RobotsStatusesManager::onMsgFlippersCurrentsReceived);
                //                subscriberFlippersPositions = NIFTiROSUtil::getNodeHandle()->subscribe(TOPIC_FLIPPERS_POSITIONS, 1, &RobotsStatusesManager::onMsgFlippersPositionsReceived);
                subscriberStatus = NIFTiROSUtil::getNodeHandle()->subscribe(TOPIC_STATUS, 1, &RobotsStatusesManager::onMsgStatusReceived);
            }

            void RobotsStatusesManager::unInit()
            {
                // Just as safety, since these objects are static, I unsubscribe here.
                subscriberFlippersCurrents.shutdown();
                //                subscriberFlippersPositions.shutdown();
                subscriberStatus.shutdown();
            }

            void RobotsStatusesManager::addListener(IRobotStatusListener* listener)
            {
                listeners.insert(listener);
            }

            void RobotsStatusesManager::removeListener(IRobotStatusListener* listener)
            {
                listeners.erase(listener);
            }

            void RobotsStatusesManager::onMsgFlippersCurrentsReceived(const nifti_robot_driver_msgs::CurrentsStampedConstPtr& msg)
            {
                flippersCurrents = msg;
            }

            //            void RobotsStatusesManager::onMsgFlippersPositionsReceived(const nifti_robot_driver_msgs::FlippersStateConstPtr& msg)
            //            {
            //                flippersPositions = msg;
            //            }

            void RobotsStatusesManager::onMsgStatusReceived(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg)
            {
                //printf("RobotsStatusesManager::onStatusMsgReceived\n");

                status = msg;

                // Calls the listeners to tell them about the new status
                for (std::set<IRobotStatusListener*>::iterator it = listeners.begin(); it != listeners.end(); it++)
                {
                    (*it)->onRobotStatusUpdated(msg);
                }
            }

            const nifti_robot_driver_msgs::CurrentsStampedConstPtr& RobotsStatusesManager::getFlippersCurrents()
            {
                return flippersCurrents;
            }

            //            const nifti_robot_driver_msgs::FlippersStateConstPtr& RobotsStatusesManager::getFlippersPositions()
            //            {
            //                return flippersPositions;
            //            }

            const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& RobotsStatusesManager::getStatus()
            {
                return status;
            }

        }
    }
}

