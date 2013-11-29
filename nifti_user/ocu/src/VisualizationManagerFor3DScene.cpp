// Benoit 2010-10-25

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include "Panels/RenderPanel.h"

#include "ViewControllers/ViewController.h"
#include "ViewControllers/ViewControllerFactory.h"

#include "NIFTiConstants.h"
#include "NIFTiROSUtil.h"

#include "VisualizationManagerFor3DScene.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            VisualizationManagerFor3DScene::VisualizationManagerFor3DScene(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams)
            : VisualizationManager(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams)
            {

            }

            VisualizationManagerFor3DScene::~VisualizationManagerFor3DScene()
            {
            }

            VisualizationManagerFor3DScene* VisualizationManagerFor3DScene::duplicate(rviz::RenderPanel* renderPanel)
            {
                VisualizationManagerFor3DScene* r = new VisualizationManagerFor3DScene(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams);
                r->initialize();
                r->setViewType(this->viewType);
                r->renderPanel->getCamera()->setPosition(this->renderPanel->getCamera()->getPosition());

                return r;
            }

            void VisualizationManagerFor3DScene::initialize()
            {
                //                renderPanel->initialize();
                //                renderPanel->setAutoRender(false);

                renderPanel->getCamera()->setNearClipDistance(0.01f);
                renderPanel->getViewport()->setBackgroundColour(NIFTiConstants::DEFAULT_BACKGROUND_COLOR_IN_OGRE_SCENE);
            }
            
            double VisualizationManagerFor3DScene::getFieldOfViewHorizontal() const
            {
                if(renderPanel->getViewport()->getCamera() == NULL) return 0; // The render panel is still not initialized
                
                if (renderPanel->getViewport()->getCamera()->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
                {
                    return 0;
                }
                else // PT_PERSPECTIVE
                {
                    // Calculation from http://en.wikipedia.org/wiki/Field_of_view_in_video_games
                    return 2 * atan(tan(getFieldOfViewVertical() / 2 * getViewport()->getActualWidth() / getViewport()->getActualHeight()));
                }
            }
                
            double VisualizationManagerFor3DScene::getFieldOfViewVertical() const
            {
                if(renderPanel->getViewport()->getCamera() == NULL) return 0; // The render panel is still not initialized
                
                if (renderPanel->getViewport()->getCamera()->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
                {
                    return 0;
                }
                else // PT_PERSPECTIVE
                {
                    return renderPanel->getViewport()->getCamera()->getFOVy().valueRadians();
                }
            }

            void VisualizationManagerFor3DScene::setViewType(ViewType viewType)
            {
                //ROS_INFO("IN void VisualizationManagerFor3DScene::setViewType(ViewType viewType) %i", viewType);

                if (!NIFTiViewsUtil::viewTypeIsVirtualScene(viewType))
                {
                    std::stringstream ss;
                    ss << "Expected a View Type of 3D Scene but received: " << viewType;
                    throw eu::nifti::misc::ExceptionWithString(ss.str());
                }
                //assert(viewTypeIsVirtualScene(viewType));

                // Returns if the requested view type is the same as the current one
                if (this->viewType == viewType) return;

                // Removes the old view controller, if necessary
                if (this->viewType != VIEW_TYPE_UNINITIALIZED)
                {
                    viewController->deactivate();
                    delete viewController;
                }

                this->viewType = viewType;

                viewController = eu::nifti::ocu::view::ViewControllerFactory::createViewController(viewType, sceneMgr, renderPanel->getCamera(), this, frameTransformer);
                viewController->activate();

                // Sets the appropriate target frame in all displays
                this->setTargetFrame(viewController->getReferenceFrameName());

                updateOCUInfoViewContent();
                updateOCUInfoViewSize();


                // Allows hiding a few things in the some views
                renderPanel->getViewport()->setVisibilityMask(NIFTiViewsUtil::getVisiblityMask(viewType));

                //ROS_INFO("OUT void VisualizationManagerFor3DScene::setViewType(ViewType viewType)");
            }

            void VisualizationManagerFor3DScene::updateOCUInfoViewContent()
            {
                if (viewParams == NULL) return; // This may mean that the PictureViewer is running and not the OCU

                //viewInfo.at(i).frameOfReference = vizMgrs.at(i)->getTargetFrame();

                viewParams->projectionType = getViewport()->getCamera()->getProjectionType();

                viewParams->fieldOfViewVertical = getFieldOfViewVertical();
                viewParams->fieldOfViewHorizontal = getFieldOfViewHorizontal();
                
                const Ogre::Vector3 p = getViewport()->getCamera()->getRealPosition();
                const Ogre::Quaternion o = getViewport()->getCamera()->getRealOrientation();

                viewParams->cameraPose.position.x = p.x;
                viewParams->cameraPose.position.y = p.y;
                viewParams->cameraPose.position.z = p.z;

                viewParams->cameraPose.orientation.w = o.w;
                viewParams->cameraPose.orientation.x = o.x;
                viewParams->cameraPose.orientation.y = o.y;
                viewParams->cameraPose.orientation.z = o.z;
            }

            void VisualizationManagerFor3DScene::updateOCUInfoViewSize()
            {
                if (viewParams == NULL) return; // This may mean that the PictureViewer is running and not the OCU

                viewParams->viewWidth = getViewport()->getActualWidth();
                viewParams->viewHeight = getViewport()->getActualHeight();

                // This could change if the window/viewport aspect ratio changed
                viewParams->fieldOfViewHorizontal = getFieldOfViewHorizontal();
            }

            // This was in pose_tool.cpp in RVIZ

            Ogre::Vector3 VisualizationManagerFor3DScene::getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const
            {
                int width = this->getViewport()->getActualWidth();
                int height = this->getViewport()->getActualHeight();

                Ogre::Ray mouse_ray = this->getViewport()->getCamera()->getCameraToViewportRay((float) mouseX / (float) width, (float) mouseY / (float) height);
                Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
                std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
                if (!intersection.first)
                {
                    return Ogre::Vector3::ZERO;
                }

                return mouse_ray.getPoint(intersection.second);
            }

            // Todo: implement

            Ogre::Image VisualizationManagerFor3DScene::getCurrentImage() const
            {
                throw "Not yet implemented";
            }

            std::string VisualizationManagerFor3DScene::getCurrentImageEncoding() const
            {
                throw "Not yet implemented";
            }

        }
    }
}
