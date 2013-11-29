// Benoit 2013-01-17

#ifndef EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_DISPLAY_H
#define EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_DISPLAY_H

#include <map>
#include <queue>
#include <set>

#include <boost/thread/mutex.hpp>

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include "InFieldPicsManager.h"

#include "Displays/Display.h"

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
 
        namespace ocu
        {
            
            namespace selection
            {
                class SelectionManager;
            }

            namespace display
            {

                class InFieldPictureMarker;
                
                typedef std::map< const std::string, const InFieldPictureMarker*> MapOfInFieldPictureMarkers;
                
                /**
                 * \class InFieldPictureDisplay
                 * \brief Displays "markers" for Objects of interest
                 *
                 */
                class InFieldPictureDisplay : public rviz::Display, eu::nifti::ocu::InFieldPicsManager::Listener
                {
                public:
                    InFieldPictureDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
                    virtual ~InFieldPictureDisplay();
                    
                    void onSelectionManagerDeleted();

                    virtual void update(float wall_dt, float ros_dt);

                    virtual void fixedFrameChanged();
                    virtual void reset();

                protected:
                    virtual void onEnable();
                    virtual void onDisable();

                    void subscribe();
                    void unsubscribe();

                    // Callbacks from the InFieldPicsManager
                    void onInFieldPicReceived(const EXIFReader_msgs::AnnotatedPicture* picture);
                    void onNewInFieldPicTaken(const EXIFReader_msgs::AnnotatedPicture* picture);

                    void addMarker(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                    void clearMarkers();

                    // Ensures that we don't read and write from the same message queue at the same time
                    boost::mutex pictureQueueMutex;

                    // Keeps all markers in a map for quick retrieval
                    MapOfInFieldPictureMarkers markers;
                    // Pictures are added to this queue as they are received, and then processed in the update() function
                    std::vector<const EXIFReader_msgs::AnnotatedPicture*> pictureQueue;

                    Ogre::SceneNode* scene_node_; ///< Scene node all the marker objects are parented to
                    
                    eu::nifti::ocu::selection::SelectionManager* selMgr;

                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_DISPLAY_H */
