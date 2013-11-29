// Benoit 2012-07-09

#ifndef EU_NIFTI_OCU_OCU_VIRTUAL_SCENE_MANAGER_H
#define EU_NIFTI_OCU_OCU_VIRTUAL_SCENE_MANAGER_H

#include "VirtualSceneManager.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Manages the virtual scene (through the DisplayManagers)
             */
            class OCUVirtualSceneManager : public VirtualSceneManager
            {
            public:

                OCUVirtualSceneManager(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::selection::SelectionManager* selectionMgr, ros::CallbackQueue* threadQueue);

                void createDisplays();
                
                void onSelectionManagerDeleted();
                
                static const std::string ROS_TOPIC_NAV_PLANNED_PATH;
                static const std::string ROS_TOPIC_TRAVELED_PATH;
                static const std::string ROS_TOPIC_MARKER_TOPO_DECOMPOSITION;
                static const std::string ROS_TOPIC_TRAVELED_PATH_MARKERS;
                static const std::string ROS_TOPIC_TIMED_POSE_MARKERS;
                static const std::string ROS_TOPIC_POSE_ARRAY_MARKERS;
                static const std::string ROS_TOPIC_FUNCTIONAL_AREAS_CARS;

            };

        }
    }
}

#endif /* EU_NIFTI_OCU_OCU_VIRTUAL_SCENE_MANAGER_H */
