// Benoit September 2010
// Inspired from RVIZ polygon_display

#ifndef NIFTI_GROUND_POLYGONS_DISPLAY_H
#define NIFTI_GROUND_POLYGONS_DISPLAY_H

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

namespace rviz
{

    /**
     * Displays multiple polygons on the ground (z=0)
     */
    class GroundPolygonsDisplay : public Display
    {
    public:
        GroundPolygonsDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        virtual ~GroundPolygonsDisplay();

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt, float ros_dt);
        virtual void reset();
        
        void setFillPolygons(bool filled);

    protected:
        void subscribe();
        void unsubscribe();
        void clear();
        void incomingMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);
        void processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);

        void modifyPolygons(const geometry_msgs::PolygonStamped::ConstPtr& msg);

        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        /**
         * Removes all polygons from the scene
         */
        void clearPolygons();

        uint32_t numPolygonsEverGenerated;
        uint32_t msgReceivedSinceLastClear;

        Ogre::SceneNode* sceneNode;

        std::set<Ogre::ManualObject*>* polygons;
        // For some reason, if I use a set instead of a set*, I get this error:
        // "glibc detected" "corrupted double-linked list"

        std::vector<Ogre::ColourValue*> colors;
        
        bool filled;

        message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_;
        tf::MessageFilter<geometry_msgs::PolygonStamped> tf_filter_;       

        static const float DEFAULT_HEIGHT;
    };

} // namespace rviz

#endif /* NIFTI_GROUND_POLYGONS_DISPLAY_H */

