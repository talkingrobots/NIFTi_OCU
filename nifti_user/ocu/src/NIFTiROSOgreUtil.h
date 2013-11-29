// Benoit 2013-08-01

#ifndef EU_NIFTI_OCU_NIFTI_ROS_OGRE_UTIL_H
#define EU_NIFTI_OCU_NIFTI_ROS_OGRE_UTIL_H

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace Ogre
{
    class Vector3;
    class Quaternion;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Contains a collection of methods to convert between ROS and Ogre types
             */
            class NIFTiROSOgreUtil
            {
            public:

                static void copyPosition(const Ogre::Vector3& source, geometry_msgs::Point& destination);
                static void copyPosition(const geometry_msgs::Point& source, Ogre::Vector3& destination);

                static void copyOrientation(const Ogre::Quaternion& source, geometry_msgs::Quaternion& destination);
                static void copyOrientation(const geometry_msgs::Quaternion& source, Ogre::Quaternion& destination);

                static void convertOrientationFromOgreToROS(Ogre::Quaternion& orientation);
                static void convertOrientationFromROSToOgre(Ogre::Quaternion& orientation);

            };

        }
    }
}



#endif  // EU_NIFTI_OCU_NIFTI_ROS_OGRE_UTIL_H
