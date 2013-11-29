// Benoit 2011-10-18


#ifndef EU_NIFTI_OCU_VIRTUAL_SCENE_MANAGER_H
#define EU_NIFTI_OCU_VIRTUAL_SCENE_MANAGER_H

#include <wx/event.h>

#include <map>

#include <ros/callback_queue.h>
#include <ros/time.h>

namespace Ogre
{
    class SceneManager;
}

class wxIdleEvent;

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
        }
    }
}

namespace rviz
{
    class Display;

    class FrameTransformer;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Manages the virtual scene (through the DisplayManagers). This is an abstract super class
             */
            class VirtualSceneManager : public wxEvtHandler
            {
            public:

                VirtualSceneManager(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::selection::SelectionManager* selectionMgr, ros::CallbackQueue* threadQueue);
                ~VirtualSceneManager();

                virtual void createDisplays() = 0;
                
                void initialize();
                
                const std::string& getFixedFrame() const;

                rviz::FrameTransformer* getFrameTransformer() const;

                void resetTime();

                void resetDisplays();

                void lockRender();

                void unlockRender();

                /**
                 * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
                 * \note This function can be called from any thread.
                 */
                void queueRender();

                void update(float wall_dt, float ros_dt);
                void onIdle(wxIdleEvent& event);

                virtual std::string toString() const;

            protected:

                /**
                 * \brief Set the coordinate frame we should be transforming all fixed data to
                 * @param frame The string name -- must match the frame name broadcast to libTF
                 */
                void setFixedFrame(const std::string& frame);
                
                /**
                 * Initializes the display
                 * @param display
                 * @param enabled
                 */
                void initDisplay(rviz::Display* display, bool enabled);

                ros::CallbackQueue* threadQueue;
                
                Ogre::SceneManager* sceneMgr;
                eu::nifti::ocu::selection::SelectionManager* selectionMgr;

                std::string fixedFrame; ///< Frame to transform fixed data to

                std::map<std::string, rviz::Display*> displays;

                boost::mutex renderMutex;
                bool renderRequested;
                ros::WallTime timeOfLastRender;

                rviz::FrameTransformer* frameTransformer;
                
                static const std::string FIXED_FRAME;
                
            };

        }
    }
}

#endif /* EU_NIFTI_OCU_VIRTUAL_SCENE_MANAGER_H */
