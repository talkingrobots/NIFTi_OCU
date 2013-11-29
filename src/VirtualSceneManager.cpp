// Benoit 2011-10-18

#include <wx/app.h>

#include <OGRE/OgreRoot.h>

#include "Displays/Display.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "VirtualSceneManager.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
 
            const std::string VirtualSceneManager::FIXED_FRAME = "/map";

            VirtualSceneManager::VirtualSceneManager(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::selection::SelectionManager* selectionMgr, ros::CallbackQueue* threadQueue)
            : threadQueue(threadQueue)
            , sceneMgr(sceneMgr)
            , selectionMgr(selectionMgr)
            , renderRequested(true)
            , frameTransformer(new rviz::FrameTransformer())
            {

            }

            VirtualSceneManager::~VirtualSceneManager()
            {
                // Deletes all displays
                std::map<std::string, rviz::Display*>::iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    delete it->second;
                }

                delete frameTransformer;
            }

            void VirtualSceneManager::initialize()
            {
                createDisplays();

                // Initializes all displays
                std::map<std::string, rviz::Display*>::iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    initDisplay(it->second, true);
                }


                setFixedFrame(FIXED_FRAME);
            }

            void VirtualSceneManager::initDisplay(rviz::Display* display, bool enabled)
            {
                display->setEnabled(enabled);
                display->setRenderCallback(boost::bind(&VirtualSceneManager::queueRender, this));
                display->setLockRenderCallback(boost::bind(&VirtualSceneManager::lockRender, this));
                display->setUnlockRenderCallback(boost::bind(&VirtualSceneManager::unlockRender, this));

                display->setFixedFrame(fixedFrame);
            }

            void VirtualSceneManager::lockRender()
            {
                renderMutex.lock();
            }

            void VirtualSceneManager::unlockRender()
            {
                renderMutex.unlock();
            }

            void VirtualSceneManager::queueRender()
            {
                if (renderRequested == false)
                {
                    wxWakeUpIdle();
                }

                renderRequested = true;
            }

            /**
             * This call comes from the main RVIZ update loop
             * @param wall_dt Difference in wall time since the last update
             * @param ros_dt Difference in ROS time since the last update
             */
            void VirtualSceneManager::update(float wall_dt, float ros_dt)
            {
                // 1) Updates the frame transformer (clears the cache)
                frameTransformer->update();

                // 2) Updates the displays
                std::map<std::string, rviz::Display*>::iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    rviz::Display* d = it->second;
                    if (d->isEnabled())
                    {
                        d->update(wall_dt, ros_dt);
                    }
                }

            }

            void VirtualSceneManager::onIdle(wxIdleEvent& evt)
            {
                ros::WallTime cur = ros::WallTime::now();
                double dt = (cur - timeOfLastRender).toSec();

                if (dt > 0.1f)
                {
                    renderRequested = true;
                }

                // Cap at 60fps
                if (renderRequested == true && dt > 0.016f)
                {
                    renderRequested = false;
                    timeOfLastRender = cur;

                    boost::mutex::scoped_lock lock(renderMutex);

                    Ogre::Root::getSingletonPtr()->renderOneFrame();
                }

                evt.Skip();
            }

            void VirtualSceneManager::resetTime()
            {
                // Resets the displays
                std::map<std::string, rviz::Display*>::iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    it->second->reset();
                }

                queueRender();
            }

            void VirtualSceneManager::setFixedFrame(const std::string& frame)
            {
                std::string remapped_name = frameTransformer->getTFClient()->resolve(frame);

                if (fixedFrame == remapped_name)
                {
                    return;
                }

                fixedFrame = remapped_name;

                frameTransformer->setFixedFrame(fixedFrame);

                std::map<std::string, rviz::Display*>::iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    it->second->setFixedFrame(fixedFrame);
                }

            }

            const std::string& VirtualSceneManager::getFixedFrame() const
            {
                return fixedFrame;
            }

            rviz::FrameTransformer* VirtualSceneManager::getFrameTransformer() const
            {
                return frameTransformer;
            }

            std::string VirtualSceneManager::toString() const
            {
                using namespace std;

                stringstream ss;

                ss << "Fixed Frame: " << fixedFrame << endl;
                ss << "Frame Transformer: " << &frameTransformer << endl;


                std::map<std::string, rviz::Display*>::const_iterator it = displays.begin();
                std::map<std::string, rviz::Display*>::const_iterator it_end = displays.end();
                for (; it != it_end; ++it)
                {
                    ss << "Display Manager \"" << it->second->getName() << "\": " << it->second->getTopic() << endl;
                }

                // Todo Display the statuses of the DisplayManagers

                return ss.str();
            }

        }
    }
}
