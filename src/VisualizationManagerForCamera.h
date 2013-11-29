// Benoit 2010-10-25

#ifndef EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_CAMERA_H_
#define EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_CAMERA_H_

#include "VisualizationManager.h"

namespace rviz
{
    class Display;
    class CameraDisplay;
    //class ImageDisplay;
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
            
            namespace view
            {
                class CameraViewController;
            }

            /**
             * Manages one render window (and everything that is displayed in it) for a camera (Omni, PTZ)
             */
            class VisualizationManagerForCamera : public rviz::VisualizationManager
            {
            public:

                VisualizationManagerForCamera(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams);
                virtual ~VisualizationManagerForCamera();

                VisualizationManagerForCamera* duplicate(rviz::RenderPanel* renderPanel);

                void initialize();
                
                double getFieldOfViewHorizontal() const;
                double getFieldOfViewVertical() const;

                void update(float wall_dt, float ros_dt);

                void resetTime();

                void setFixedFrame(const std::string& frame);

                void setViewType(ViewType viewType);

                /**
                 * Forces the display to render at the next call to update
                 */
                void forceReRender();

                void updateOCUInfoViewContent();
                void updateOCUInfoViewSize();
                
                void handleSizeEvent(wxSizeEvent& evt);

                Ogre::Vector3 getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const;
                
                Ogre::Image getCurrentImage() const;
                
                std::string getCurrentImageEncoding() const;

            protected:

                void initDisplay(rviz::Display* display, bool enabled);

                void setupKinectP3AT();
                void setupPTZ();
                void setupOmni();
                void setupVirtualPTZWithoutOverlay();
                void setupVirtualPTZWithOverlay();
                void setupArmCam();
                void setupUAVCamDown();
                void setupUAVCamFront();

                // Simply an additional pointer to "viewController" that allows calling functions specific to CameraViewController
                eu::nifti::ocu::view::CameraViewController* cameraViewController;
                
                rviz::CameraDisplay* display;
                //rviz::ImageDisplay* display;
                

            };

        }
    }
}

#endif /* EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_CAMERA_H_ */
