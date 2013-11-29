// Benoit 2011-03-23
// Inspired from RVIZ polygon_display

#ifndef NIFTI_POSE_DISPLAY_H
#define NIFTI_POSE_DISPLAY_H

#include <vector>
#include <set>

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
                 * Displays the footprint of the robot as the pose information
                 */
                class NIFTiPoseDisplay : public rviz::Display
                {
                public:
                    NIFTiPoseDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
                    virtual ~NIFTiPoseDisplay();

                    // Overrides from Display
                    virtual void fixedFrameChanged();
                    virtual void update(float wall_dt, float ros_dt);
                    virtual void reset();

                protected:
                    void subscribe();
                    void unsubscribe();
                    void clear();
                    void processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);

                    // overrides from Display
                    virtual void onEnable();
                    virtual void onDisable();

                    uint32_t msgReceivedSinceLastClear;

                    Ogre::SceneNode* sceneNode;
                    Ogre::ManualObject* posePolygon;

                    message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_;
                    tf::MessageFilter<geometry_msgs::PolygonStamped> tf_filter_;

                    static const Ogre::ColourValue COLOR;

                    static const char* ROS_TOPIC;
                    static const float DEFAULT_HEIGHT;
                };

            }
        }
    }
}

#endif /* NIFTI_POSE_DISPLAY_H */

