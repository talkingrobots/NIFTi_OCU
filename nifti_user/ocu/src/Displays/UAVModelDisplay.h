// Benoit 2012-11-20

#ifndef EU_NIFTI_OCU_UAV_MODEL_DISPLAY_H
#define EU_NIFTI_OCU_UAV_MODEL_DISPLAY_H

#include <geometry_msgs/PolygonStamped.h>

#include "Displays/Display.h"

namespace Ogre
{
    class SceneNode;
    class ManualObject;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                /**
                 * Displays a simplistic 3D model of the UAV
                 */
                class UAVModelDisplay : public rviz::Display
                {
                public:
                    UAVModelDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
                    virtual ~UAVModelDisplay();

                    // Overrides from Display
                    virtual void fixedFrameChanged();
                    virtual void update(float wall_dt, float ros_dt);
                    
                protected:
                    void incomingMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);

                    void createOgreModel();
                    void updatePositionOfSceneNode();

                    // overrides from Display
                    virtual void onEnable();
                    virtual void onDisable();

                    Ogre::SceneNode* sceneNode;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_UAV_MODEL_DISPLAY_H */

