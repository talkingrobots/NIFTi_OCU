// Benoit 2011-11-17

#ifndef EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_BLANK_SCREEN_H_
#define EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_BLANK_SCREEN_H_

#include "IVisualizationManager.h"

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

            namespace selection
            {
                class SelectionManager;
            }

            /**
             * Manages one render window by making it blank. Useful for newly
             * created window before the user puts something else in it.
             */
            class VisualizationManagerForBlankScreen : public IVisualizationManager
            {
            public:

                VisualizationManagerForBlankScreen(rviz::RenderPanel* renderPanel, nifti_ocu_msgs::NIFTiViewParams* viewParams);
                ~VisualizationManagerForBlankScreen();

                VisualizationManagerForBlankScreen* duplicate(rviz::RenderPanel* renderPanel);

                void initialize();

                double getFieldOfViewHorizontal() const;
                double getFieldOfViewVertical() const;

                const std::string& getFixedFrame() const;


                const std::string& getTargetFrame() const;

                void resetTime();


                Ogre::Viewport* getViewport() const;


                eu::nifti::ocu::view::IViewController* getViewController() const;


                void lockRender();


                void unlockRender();


                void queueRender();

                void forceReRender();

                void update(float wall_dt, float ros_dt);

                void onIdle(wxIdleEvent& event);


                void lookAt(const Ogre::Vector3& point);

                void updateOCUInfoViewContent();

                void updateOCUInfoViewSize();

                void handleSizeEvent(wxSizeEvent& evt);

                const rviz::RenderPanel* getRenderPanel() const;

                Ogre::Vector3 getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const;


                ViewType getViewType() const;

                void setViewType(ViewType viewType);

                void changeViewParams(nifti_ocu_msgs::NIFTiViewParams* viewParams);

                Ogre::Image getCurrentImage() const;

                std::string getCurrentImageEncoding() const;

                std::string toString() const;


            protected:
                const rviz::RenderPanel* renderPanel;

                eu::nifti::ocu::view::IViewController* viewController;

                nifti_ocu_msgs::NIFTiViewParams* viewParams;

            private:
                std::string s;
            };

        }
    }
}

#endif /* EU_NIFTI_OCU_VISUALIZATION_MANAGER_FOR_BLANK_SCREEN_H_ */
