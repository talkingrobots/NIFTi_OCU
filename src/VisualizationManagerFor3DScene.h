// Benoit 2010-10-25

#ifndef EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_VIRTUAL_SCENE_H_
#define EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_VIRTUAL_SCENE_H_

#include "VisualizationManager.h"

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

            /**
             * Manages one render window (and everything that is displayed in it) 
             * for the virtual scene. In particular, handles the view point
             * controllers
             */
            class VisualizationManagerFor3DScene : public rviz::VisualizationManager
            {
            public:

                VisualizationManagerFor3DScene(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, rviz::RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams);
                virtual ~VisualizationManagerFor3DScene();

                VisualizationManagerFor3DScene* duplicate(rviz::RenderPanel* renderPanel);

                void initialize();
                
                double getFieldOfViewHorizontal() const;
                double getFieldOfViewVertical() const;

                void setViewType(ViewType viewType);

                void updateOCUInfoViewContent();
                void updateOCUInfoViewSize();
                
                Ogre::Image getCurrentImage() const;
                
                std::string getCurrentImageEncoding() const;

                Ogre::Vector3 getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const;

            };

        }
    }
}

#endif /* EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_VIRTUAL_SCENE_H_ */
