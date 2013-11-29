// Benoit 2013-08-01

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "NIFTiROSOgreUtil.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            void NIFTiROSOgreUtil::copyPosition(const Ogre::Vector3& source, geometry_msgs::Point& destination)
            {
                destination.x = source.x;
                destination.y = source.y;
                destination.z = source.z;
            }

            void NIFTiROSOgreUtil::copyPosition(const geometry_msgs::Point& source, Ogre::Vector3& destination)
            {
                destination.x = source.x;
                destination.y = source.y;
                destination.z = source.z;
            }

            void NIFTiROSOgreUtil::copyOrientation(const Ogre::Quaternion& source, geometry_msgs::Quaternion& destination)
            {
                destination.w = source.w;
                destination.x = source.x;
                destination.y = source.y;
                destination.z = source.z;
            }

            void NIFTiROSOgreUtil::copyOrientation(const geometry_msgs::Quaternion& source, Ogre::Quaternion& destination)
            {
                destination.w = source.w;
                destination.x = source.x;
                destination.y = source.y;
                destination.z = source.z;
            }

            /**
             * In this case, the orientation is taken from the Ogre camera, where x goes right, y goes up, and z goes
             * into the image. It's not pointing in the same way as with TFs, so we need to turn it.
             * @param source
             */
            void NIFTiROSOgreUtil::convertOrientationFromOgreToROS(Ogre::Quaternion& orientation)
            {
                // http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Quaternion+and+Rotation+Primer
                // Rotates the quaternion -90 deg along the x axis and 90 deg along the z axis
                Ogre::Quaternion rotXNeg(sqrt(0.5), -sqrt(0.5), 0, 0);
                Ogre::Quaternion rotZ(sqrt(0.5), 0, 0, sqrt(0.5));

                orientation = orientation * rotXNeg * rotZ;
            }

            void NIFTiROSOgreUtil::convertOrientationFromROSToOgre(Ogre::Quaternion& orientation)
            {
                // http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Quaternion+and+Rotation+Primer
                // Rotates the quaternion 90 deg along the x axis and -90 deg along the z axis
                Ogre::Quaternion rotXNeg(-sqrt(0.5), -sqrt(0.5), 0, 0);
                Ogre::Quaternion rotZ(-sqrt(0.5), 0, 0, sqrt(0.5));

                orientation = orientation * rotZ * rotXNeg;
            }

        }
    }
}

