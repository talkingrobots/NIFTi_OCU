// Benoit 2011-06-08

#ifndef EU_NIFTI_DISPLAY_OOI_MARKER_FACTORY_H
#define EU_NIFTI_DISPLAY_OOI_MARKER_FACTORY_H

#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

#include "Displays/Markers/ObjectOfInterestMarker.h"

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

            namespace display
            {

                class OOIMarkerFactory
                {
                public:
                    static ObjectOfInterestMarkerPtr getOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);
                    static void initialize(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* sceneNode);

                private:
                    OOIMarkerFactory(); // This constructor is private, because I want this to be an abstract class

                    static Ogre::SceneManager* sceneMgr;
                    static rviz::FrameTransformer* frameTransformer;
                    static Ogre::SceneNode* sceneNode;

                };

            }
        }
    }
}

#endif // EU_NIFTI_DISPLAY_OOI_MARKER_FACTORY_H
