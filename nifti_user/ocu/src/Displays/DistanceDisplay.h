// Benoit 2012-06-22

#ifndef NIFTI_DISTANCE_DISPLAY_H
#define NIFTI_DISTANCE_DISPLAY_H

#include <geometry_msgs/PolygonStamped.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

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
                 * Displays multiple polygons on the ground (z=0)
                 */
                class DistanceDisplay : public rviz::Display
                {
                public:
                    DistanceDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
                    virtual ~DistanceDisplay();

                    // Overrides from Display
                    virtual void fixedFrameChanged();
                    virtual void update(float wall_dt, float ros_dt);
                    
                protected:
                    void incomingMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);
                    
                    void createCircles();
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

#endif /* NIFTI_DISTANCE_DISPLAY_H */

