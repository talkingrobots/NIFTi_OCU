// Benoit 2011-06-30

#ifndef EU_NIFTI_ENV_MSG_ROS_UTIL_H
#define EU_NIFTI_ENV_MSG_ROS_UTIL_H

#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

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

            namespace msg
            {

                namespace ros
                {

                    class Util
                    {
                    public:
                        static eu::nifti::env::UUID getUUID(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);
                        static const eu_nifti_env::ObjectOfInterest* getOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);

                    private:
                        Util(); // This constructor is private, because I want this to be an abstract class

                    };

                }
            }
        }
    }
}

#endif // EU_NIFTI_ENV_MSG_ROS_UTIL_H
