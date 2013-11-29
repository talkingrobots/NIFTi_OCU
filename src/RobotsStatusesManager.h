// Benoit 2011-09-01

#ifndef EU_NIFTI_OCU_ROBOTS_STATUSES_MANAGER_H
#define EU_NIFTI_OCU_ROBOTS_STATUSES_MANAGER_H

#include <set>

#include <ros/subscriber.h>

#include <nifti_robot_driver_msgs/CurrentsStamped.h>
//#include <nifti_robot_driver_msgs/FlippersState.h>
#include <nifti_robot_driver_msgs/RobotStatusStamped.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IRobotStatusListener;

            /**
             * Listens on ROS and manages all statuses from the various robots
             * on the network. Maintains a list of listeners who want to be
             * notified of the latest update.
             */
            class RobotsStatusesManager
            {
            public:
                
                static const char* TOPIC_FLIPPERS_CURRENTS;
                //static const char* TOPIC_FLIPPERS_POSITIONS;
                static const char* TOPIC_STATUS;

                static void init();
                static void unInit();
                
                static void addListener(IRobotStatusListener* listener);
                static void removeListener(IRobotStatusListener* listener);
                
                static const nifti_robot_driver_msgs::CurrentsStampedConstPtr& getFlippersCurrents();
                //static const nifti_robot_driver_msgs::FlippersStateConstPtr& getFlippersPositions();
                static const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& getStatus();
                
            protected:

                static void onMsgFlippersCurrentsReceived(const nifti_robot_driver_msgs::CurrentsStampedConstPtr& msg);
                //static void onMsgFlippersPositionsReceived(const nifti_robot_driver_msgs::FlippersStateConstPtr& msg);
                static void onMsgStatusReceived(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg);
                
                static std::set<IRobotStatusListener*> listeners;
                
                static ros::Subscriber subscriberFlippersCurrents;
                //static ros::Subscriber subscriberFlippersPositions;
                static ros::Subscriber subscriberStatus;
                
                static nifti_robot_driver_msgs::CurrentsStampedConstPtr flippersCurrents;
                //static nifti_robot_driver_msgs::FlippersStateConstPtr flippersPositions;
                static nifti_robot_driver_msgs::RobotStatusStampedConstPtr status;
                
            };
            
            

        }
    }
}



#endif
