// Benoit 2010-10-25

#include <string>

#include <OgreCamera.h>
#include <OgreViewport.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include "NIFTiConstants.h"
#include "NIFTiROSOgreUtil.h"
#include "NIFTiROSUtil.h"

#include "Displays/CameraDisplay.h"
//#include "Displays/ImageDisplay.h"

#include "Panels/RenderPanel.h"

#include "ViewControllers/ViewController.h"
#include "ViewControllers/ViewControllerFactory.h"

#include "VisualizationManagerForCamera.h"
#include "ViewControllers/CameraViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const std::string DEFAULT_TRANSPORT_MODE = "theora"; // Parameter: imageTransportMode

            const std::string DISPLAY_CAMERA_NAME = "Camera";

            const std::string FIXED_FRAME_P3_AT_KINECT = "/camera_rgb_optical_frame";
            const std::string FIXED_FRAME_OMNI_CAM = "/pano";
            const std::string FIXED_FRAME_PTZ_CAM = "/head_camera";
            //const std::string FIXED_FRAME_VIRTUAL_PTZ_CAM = "/processed_image";
            const std::string FIXED_FRAME_VIRTUAL_PTZ_CAM = "/map";
            //const std::string FIXED_FRAME_ARM_CAM = "/head_camera";
            //const std::string FIXED_FRAME_ARM_CAM = "/thermo_camera";
            const std::string FIXED_FRAME_ARM_CAM = "/map";
            const std::string FIXED_FRAME_UAV_CAM_DOWN = "/uav_cam_down";
            const std::string FIXED_FRAME_UAV_CAM_FRONT = "/uav_cam_front";

            // Configuration for the Kinect on the P3-AT
            const float CAM_P3_AT_KINECT_ALPHA = 1;
            const char* CAM_P3_AT_KINECT_TOPIC = "/camera/rgb/image_color";

            // Configuration for a small USB webcam
            const float CAM_USB_ALPHA = 0.75;
            const char* CAM_USB_TOPIC = "/viz/PTZ";

            // Configuration for the Ladybug3 omni-cam
            const float CAM_OMNI_ALPHA = 1; // This prevents the semi-transparent virtual scene to be overlayed (not possible anyway for omni-views)
            const char* CAM_OMNI_TOPIC = "/viz/pano/image";

            // Configuration for the Ladybug3 virtual PTZ
            const float CAM_VIRTUAL_PTZ_ALPHA_WITHOUT_OVERLAY = 1;
            const char* CAM_VIRTUAL_PTZ_TOPIC = "/viz/ptz/image";
            //const char* CAM_VIRTUAL_PTZ_TOPIC = "/viz/ptz/processed_image";

            const float ARM_CAM_ALPHA = 0.75;
            //const char* ARM_CAM_TOPIC = "/camera/rgb/image_color";
            const char* ARM_CAM_TOPIC = "/openni_camera/detections";
            //const char* ARM_CAM_TOPIC = "/openni_camera/rgb";

            // Configuration for the UAV camera that points down
            const float CAM_UAV_DOWN_ALPHA = 1; // This prevents the semi-transparent virtual scene to be overlayed (the TFs are not yet correct)
            const char* CAM_UAV_DOWN_TOPIC = "/viz/uav_cam_down/image";

            // Configuration for the UAV camera that points forward
            const float CAM_UAV_FRONT_ALPHA = 1; // This prevents the semi-transparent virtual scene to be overlayed (the TFs are not yet correct)
            const char* CAM_UAV_FRONT_TOPIC = "/viz/uav_cam_front/image";

            VisualizationManagerForCamera::VisualizationManagerForCamera(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams)
            : VisualizationManager(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams)
            , display(NULL)
            {

            }

            VisualizationManagerForCamera::~VisualizationManagerForCamera()
            {
                //std::cout << "IN VisualizationManagerForCamera::~VisualizationManagerForCamera()" << std::endl;

                delete display;
                //delete frameTransformer;

                //std::cout << "OUT VisualizationManagerForCamera::~VisualizationManagerForCamera()" << std::endl;
            }

            VisualizationManagerForCamera* VisualizationManagerForCamera::duplicate(rviz::RenderPanel* renderPanel)
            {
                VisualizationManagerForCamera* r = new VisualizationManagerForCamera(sceneMgr, frameTransformer, selectionMgr, renderPanel, threadedQueue, viewParams);
                r->initialize();
                r->setViewType(this->viewType);

                return r;
            }

            // No default view is provided. The user MUST call setViewType after

            void VisualizationManagerForCamera::initialize()
            {
                //                renderPanel->initialize();
                //                renderPanel->setAutoRender(false);

                // Todo Adjust these params
                renderPanel->getViewport()->setBackgroundColour(NIFTiConstants::DEFAULT_BACKGROUND_COLOR_IN_OGRE_SCENE);

                display = new rviz::CameraDisplay(DISPLAY_CAMERA_NAME, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadedQueue, renderPanel->getCamera(), renderPanel->getRenderWindow(), renderPanel->getViewport());
                //display = new rviz::ImageDisplay(DISPLAY_CAMERA_NAME, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadedQueue, renderPanel);

                initDisplay(display, true);

                // Benoit: it seems better if no default is provided, otherwise ROS complains if the user changes this default within a millisecond since we start listening on a topic and stop right after
                //this->setViewType(VIEW_TYPE_OMNI_CAM); 

                // The target frame never changes for the camera display, so we
                // set it just once here.
                setTargetFrame(NIFTiConstants::FIXED_FRAME_STRING);

                // Sets the transport to default or the parameter if found
                std::string transportMode;
                if (NIFTiROSUtil::getParam("imageTransportMode", transportMode))
                {
                    assert(transportMode == "raw" || transportMode == "compressed" || transportMode == "theora");
                    display->setTransport(transportMode);
                }
                else
                {
                    display->setTransport(DEFAULT_TRANSPORT_MODE);
                }

            }

            double VisualizationManagerForCamera::getFieldOfViewHorizontal() const
            {
                return viewController->getFieldOfViewHorizontal();
            }

            double VisualizationManagerForCamera::getFieldOfViewVertical() const
            {
                return viewController->getFieldOfViewVertical();
            }

            /**
             * This call comes from the main RVIZ update loop
             * @param wall_dt Difference in wall time since the last update
             * @param ros_dt Difference in ROS time since the last update
             */
            void VisualizationManagerForCamera::update(float wall_dt, float ros_dt)
            {
                VisualizationManager::update(wall_dt, ros_dt);

                if (display->isEnabled())
                {
                    display->update(wall_dt, ros_dt);
                    //display->printOutFieldOfViews();
                }
            }

            void VisualizationManagerForCamera::resetTime()
            {
                display->reset();

                VisualizationManager::resetTime();
            }

            void VisualizationManagerForCamera::setFixedFrame(const std::string& frame)
            {
                VisualizationManager::setFixedFrame(frame);

                display->setFixedFrame(frame);

            }

            void VisualizationManagerForCamera::forceReRender()
            {
                if (this->display == NULL) return;

                this->display->forceRender();
            }

            // Todo Benoit Improve setViewType to get rid of the ROS_WARN message Timer destroyed immediately after creation.  Did you forget to store the handle? The problem comes at [deleting (boost reset) the TF Filter]

            void VisualizationManagerForCamera::setViewType(ViewType viewType) // throws image_transport::TransportLoadException
            {
                if (!NIFTiViewsUtil::viewTypeIsCamera(viewType))
                {
                    std::stringstream ss;
                    ss << "Expected a View Type of Camera but received: " << viewType;
                    throw eu::nifti::misc::ExceptionWithString(ss.str());
                }
                //assert(viewTypeIsCam(viewType));


                // Returns if the requested view type is the same as the current one
                if (this->viewType == viewType) return;

                bool changeVideoFeed = !((this->viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM && viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY) || (this->viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY && viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM));
                // Nothing to do because we're staying with the same camera feed

                if (changeVideoFeed)
                {
                    // Removes the old view controller, if necessary
                    if (this->viewType != VIEW_TYPE_UNINITIALIZED)
                    {
                        viewController->deactivate();
                        delete viewController;
                    }

                    // todo catch the exception thrown in this method
                    viewController = eu::nifti::ocu::view::ViewControllerFactory::createViewController(viewType, sceneMgr, renderPanel->getCamera(), this, frameTransformer);
                    cameraViewController = dynamic_cast<eu::nifti::ocu::view::CameraViewController*> (viewController); // Copies the pointer
                    assert(viewController);
                    viewController->activate();
                }

                this->viewType = viewType;

                if (viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM)
                {
                    setupVirtualPTZWithoutOverlay();
                }
                else if (viewType == VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY)
                {
                    setupVirtualPTZWithOverlay();
                }
                else if (viewType == VIEW_TYPE_OMNI_CAM)
                {
                    setupOmni();
                }
                else if (viewType == VIEW_TYPE_PTZ_CAM)
                {
                    setupPTZ();
                }
                else if (viewType == VIEW_TYPE_KINECT_CAM)
                {
                    setupKinectP3AT();
                }
                else if (viewType == VIEW_TYPE_ARM_CAM)
                {
                    setupArmCam();
                }
                else if (viewType == VIEW_TYPE_UAV_CAM_DOWN)
                {
                    setupUAVCamDown();
                }
                else if (viewType == VIEW_TYPE_UAV_CAM_FRONT)
                {
                    setupUAVCamFront();
                }
                else
                {
                    // This will never happen
                    assert(false);
                }

                // Allows hiding a few things in the Virtual PTZ view
                renderPanel->getViewport()->setVisibilityMask(NIFTiViewsUtil::getVisiblityMask(viewType));

                forceReRender();

                updateOCUInfoViewContent();
                updateOCUInfoViewSize();
            }

            void VisualizationManagerForCamera::setupKinectP3AT()
            {
                setFixedFrame(FIXED_FRAME_P3_AT_KINECT);
                display->setAlpha(CAM_P3_AT_KINECT_ALPHA);
                display->setTopic(CAM_P3_AT_KINECT_TOPIC);
            }

            void VisualizationManagerForCamera::setupPTZ()
            {
                setFixedFrame(FIXED_FRAME_PTZ_CAM);
                display->setAlpha(CAM_USB_ALPHA);
                display->setTopic(CAM_USB_TOPIC);
            }

            void VisualizationManagerForCamera::setupOmni()
            {
                setFixedFrame(FIXED_FRAME_OMNI_CAM);
                display->setAlpha(CAM_OMNI_ALPHA);
                display->setTopic(CAM_OMNI_TOPIC);
            }

            void VisualizationManagerForCamera::setupVirtualPTZWithoutOverlay()
            {
                //ROS_INFO("setupVirtualPTZWithoutOverlay");
                setFixedFrame(FIXED_FRAME_VIRTUAL_PTZ_CAM);
                display->setAlpha(CAM_VIRTUAL_PTZ_ALPHA_WITHOUT_OVERLAY);
                display->setImagePosition(rviz::CameraDisplay::IMAGE_POS_OVERLAY);
                display->setTopic(CAM_VIRTUAL_PTZ_TOPIC);
            }

            void VisualizationManagerForCamera::setupVirtualPTZWithOverlay()
            {
                //ROS_INFO("setupVirtualPTZWithOverlay");
                setFixedFrame(FIXED_FRAME_VIRTUAL_PTZ_CAM);
                display->setImagePosition(rviz::CameraDisplay::IMAGE_POS_BACKGROUND);
                display->setTopic(CAM_VIRTUAL_PTZ_TOPIC);
            }

            void VisualizationManagerForCamera::setupArmCam()
            {
                setFixedFrame(FIXED_FRAME_ARM_CAM);
                display->setAlpha(ARM_CAM_ALPHA);
                display->setImagePosition(rviz::CameraDisplay::IMAGE_POS_OVERLAY);
                display->setTopic(ARM_CAM_TOPIC);
            }

            void VisualizationManagerForCamera::setupUAVCamDown()
            {
                setFixedFrame(FIXED_FRAME_UAV_CAM_DOWN);
                display->setAlpha(CAM_UAV_DOWN_ALPHA);
                display->setTopic(CAM_UAV_DOWN_TOPIC);
            }

            void VisualizationManagerForCamera::setupUAVCamFront()
            {
                setFixedFrame(FIXED_FRAME_UAV_CAM_FRONT);
                display->setAlpha(CAM_UAV_FRONT_ALPHA);
                display->setTopic(CAM_UAV_FRONT_TOPIC);
            }

            void VisualizationManagerForCamera::updateOCUInfoViewContent()
            {
                viewParams->projectionType = -1; // Not applicable in this case

                Ogre::Vector3 position;
                Ogre::Quaternion orientation;

                try
                {
                    position = cameraViewController->getPosition();
                    orientation = cameraViewController->getOrientation();
                }
                catch (const std::string& ex)
                {
                    ROS_DEBUG_STREAM("Cannot get a position for the CameraViewController with ViewType " << viewType << "." << ex);
                }

                NIFTiROSOgreUtil::copyPosition(position, viewParams->cameraPose.position);
                NIFTiROSOgreUtil::copyOrientation(orientation, viewParams->cameraPose.orientation);
                viewParams->fieldOfViewHorizontal = viewController->getFieldOfViewHorizontal();
                viewParams->fieldOfViewVertical = viewController->getFieldOfViewVertical();
            }

            void VisualizationManagerForCamera::updateOCUInfoViewSize()
            {
                viewParams->viewWidth = getViewport()->getActualWidth();
                viewParams->viewHeight = getViewport()->getActualHeight();
            }

            void VisualizationManagerForCamera::initDisplay(rviz::Display* display, bool enabled)
            {
                display->setEnabled(enabled);
                display->setRenderCallback(boost::bind(&VisualizationManager::queueRender, this));
                display->setLockRenderCallback(boost::bind(&VisualizationManager::lockRender, this));
                display->setUnlockRenderCallback(boost::bind(&VisualizationManager::unlockRender, this));

                display->setFixedFrame(getFixedFrame());
            }

            void VisualizationManagerForCamera::handleSizeEvent(wxSizeEvent& evt)
            {
                forceReRender();
            }

            Ogre::Vector3 VisualizationManagerForCamera::getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const
            {
                // Todo Implement me
                assert(false);
            }

            Ogre::Image VisualizationManagerForCamera::getCurrentImage() const
            {
                return display->getCurrentImage();
            }

            std::string VisualizationManagerForCamera::getCurrentImageEncoding() const
            {
                return display->getCurrentImageEncoding();
            }

        }
    }
}
