// Benoit 2011-09-01

#ifndef EU_NIFTI_OCU_ROBOT_STATUS_LISTENER_H
#define EU_NIFTI_OCU_ROBOT_STATUS_LISTENER_H

#include <nifti_robot_driver_msgs/RobotStatusStamped.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            /**
             * Interface of a class that listens to robot statuses
             */
            class IRobotStatusListener
            {
            public:
                virtual void onRobotStatusUpdated(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg) = 0;
            };                    
            
        }
    }
}



#endif // EU_NIFTI_OCU_ROBOT_STATUS_LISTENER_H
