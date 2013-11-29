// Benoit 2010-11-19

#ifndef NIFTI_GENERIC_MARKER_CREATOR_H
#define NIFTI_GENERIC_MARKER_CREATOR_H

#include "Displays/Markers/marker_base.h"

namespace Ogre
{
    class SceneManager;
    class SceneNode;
}

namespace rviz
{

    class FrameTransformer;
    class MarkerDisplay;

    /**
     * This class is a singleton that creates markers, like the original RVIZ
     * @return
     */
    class GenericMarkerCreator
    {
    public:
        static GenericMarkerCreator* instance();
        rviz::MarkerBase* createMarker(const visualization_msgs::Marker::ConstPtr& message, rviz::MarkerDisplay* owner, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* sceneNode) const;

    private:
        GenericMarkerCreator(); // This class is a singleton
        static GenericMarkerCreator* theInstance;
    };

}

#endif // NIFTI_GENERIC_MARKER_CREATOR_H
