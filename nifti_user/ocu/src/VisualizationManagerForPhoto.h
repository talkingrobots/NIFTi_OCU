// Benoit 2013-03-27

#ifndef EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_PHOTO_H_
#define EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_PHOTO_H_

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include "VisualizationManager.h"

namespace rviz
{
    class RenderPanel;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {
                class PhotoViewController;
            }
            
            namespace display
            {
                class PhotoDisplay;
            }

            /**
             * Manages one render window (and everything that is displayed in it) for a camera (Omni, PTZ)
             */
            class VisualizationManagerForPhoto : public rviz::VisualizationManager
            {
            public:

                VisualizationManagerForPhoto(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams);
                virtual ~VisualizationManagerForPhoto();

                VisualizationManagerForPhoto* duplicate(rviz::RenderPanel* renderPanel);

                void initialize();
                
                double getFieldOfViewHorizontal() const;
                double getFieldOfViewVertical() const;

                void update(float wall_dt, float ros_dt);

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
                
                void setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture);

            protected:

                void initDisplay(rviz::Display* display, bool enabled);

                // Simply an additional pointer to "viewController" that allows calling functions specific to CameraViewController
                eu::nifti::ocu::view::PhotoViewController* photoViewController;
                
                eu::nifti::ocu::display::PhotoDisplay* display;

            };
            
        }
    }
}

#endif /* EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_PHOTO_H_ */
