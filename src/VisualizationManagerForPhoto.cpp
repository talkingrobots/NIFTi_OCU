// Benoit 2013-03-27

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreViewport.h>

#include "NIFTiConstants.h"

#include "Displays/PhotoDisplay.h"

#include "Panels/RenderPanel.h"

#include "ViewControllers/PhotoViewController.h"

#include "VisualizationManagerForPhoto.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            VisualizationManagerForPhoto::VisualizationManagerForPhoto(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams)
            : VisualizationManager(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams)
            {
            }

            VisualizationManagerForPhoto::~VisualizationManagerForPhoto()
            {
                //renderPanel->getCamera()->setFarClipDistance(100000); // This seems to be the default
                
                renderPanel->getRenderWindow()->setActive(true);
                renderPanel->getRenderWindow()->setAutoUpdated(true);

                renderPanel->getRenderWindow()->removeListener(display);

                delete display;
            }

            VisualizationManagerForPhoto* VisualizationManagerForPhoto::duplicate(rviz::RenderPanel* renderPanel)
            {
                VisualizationManagerForPhoto* r = new VisualizationManagerForPhoto(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams);
                r->initialize();

                return r;
            }

            void VisualizationManagerForPhoto::initialize()
            {

                // Customizes the render panel for displaying photos
                {
                    renderPanel->getViewport()->setBackgroundColour(Ogre::ColourValue::Black);

                    // Benoit: I removed the ones that did not seem to have any effect
                    //viewPort->setOverlaysEnabled(false);
                    //viewPort->setClearEveryFrame(true);

                    //                renderPanel->initialize();
                    //                renderPanel->setAutoRender(false);
                    renderPanel->getRenderWindow()->setActive(false);
                    renderPanel->getRenderWindow()->setAutoUpdated(false);

                    // This is a hack, but it ensures that nothing else than the photo gets shown
                    // I can't find another simple way to hide everything except the photo
                    renderPanel->getCamera()->setPosition(-999999, -999999, -999999);
                }


                display = new eu::nifti::ocu::display::PhotoDisplay("Photo", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadedQueue, renderPanel->getRenderWindow());

                initDisplay(display, true);

                // we use this to make this node always invisible except when we draw in this particular renderWindow
                renderPanel->getRenderWindow()->addListener(display);

                viewType = eu::nifti::ocu::VIEW_TYPE_PHOTO;

                setTargetFrame(NIFTiConstants::FIXED_FRAME_STRING);

                photoViewController = new eu::nifti::ocu::view::PhotoViewController(display);
                viewController = photoViewController;
                viewController->activate();

                updateOCUInfoViewContent();
                updateOCUInfoViewSize();
            }
            
            double VisualizationManagerForPhoto::getFieldOfViewHorizontal() const
            {
                return 0;
            }

            double VisualizationManagerForPhoto::getFieldOfViewVertical() const
            {
                return 0;
            }

            /**
             * This call comes from the main RVIZ update loop
             * @param wall_dt Difference in wall time since the last update
             * @param ros_dt Difference in ROS time since the last update
             */
            void VisualizationManagerForPhoto::update(float wall_dt, float ros_dt)
            {
                VisualizationManager::update(wall_dt, ros_dt);

                display->update(wall_dt, ros_dt);
            }

            void VisualizationManagerForPhoto::forceReRender()
            {
                if (this->display == NULL) return;

                this->display->forceRender();
            }

            void VisualizationManagerForPhoto::setViewType(ViewType viewType)
            {

            }

            void VisualizationManagerForPhoto::updateOCUInfoViewContent()
            {
                viewParams->projectionType = -1; // Not applicable in this case

                viewParams->cameraPose.position = geometry_msgs::Point();
                viewParams->cameraPose.orientation = geometry_msgs::Quaternion();
                viewParams->fieldOfViewHorizontal = 0;
                viewParams->fieldOfViewVertical = 0;
            }

            void VisualizationManagerForPhoto::updateOCUInfoViewSize()
            {
                viewParams->viewWidth = getViewport()->getActualWidth();
                viewParams->viewHeight = getViewport()->getActualHeight();
            }

            void VisualizationManagerForPhoto::initDisplay(rviz::Display* display, bool enabled)
            {
                //display->setEnabled(enabled);
                display->setRenderCallback(boost::bind(&VisualizationManager::queueRender, this));
                display->setLockRenderCallback(boost::bind(&VisualizationManager::lockRender, this));
                display->setUnlockRenderCallback(boost::bind(&VisualizationManager::unlockRender, this));

                //display->setFixedFrame(getFixedFrame());
            }
            
            void VisualizationManagerForPhoto::handleSizeEvent(wxSizeEvent& evt)
            {
                display->handleSizeEvent(evt);
            }

            Ogre::Vector3 VisualizationManagerForPhoto::getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const
            {
                return Ogre::Vector3();
            }

            Ogre::Image VisualizationManagerForPhoto::getCurrentImage() const
            {
                throw "Not yet implemented";
            }
            
            std::string VisualizationManagerForPhoto::getCurrentImageEncoding() const
            {
                throw "Not yet implemented";
            }

            void VisualizationManagerForPhoto::setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture)
            {
                display->setPhoto(picture);
            }


        }
    }
}
