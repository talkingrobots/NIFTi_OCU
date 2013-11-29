// Benoit 2011-11-17

#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreViewport.h>

#include "Panels/RenderPanel.h"

#include "nifti_ocu_msgs/NIFTiViewParams.h"

#include "ViewControllers/IViewController.h"
#include "ViewControllers/ViewControllerFactory.h"

#include "VisualizationManagerForBlankScreen.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            VisualizationManagerForBlankScreen::VisualizationManagerForBlankScreen(rviz::RenderPanel* renderPanel, nifti_ocu_msgs::NIFTiViewParams* viewParams)
            : renderPanel(renderPanel)
            , viewParams(viewParams)
            {

            }

            VisualizationManagerForBlankScreen::~VisualizationManagerForBlankScreen()
            {
                renderPanel->getRenderWindow()->setAutoUpdated(true);

                viewController->deactivate();
                delete viewController;
            }

            VisualizationManagerForBlankScreen* VisualizationManagerForBlankScreen::duplicate(rviz::RenderPanel* renderPanel)
            {
                return NULL;
            }

            void VisualizationManagerForBlankScreen::initialize()
            {
                renderPanel->getRenderWindow()->setAutoUpdated(false);

                // Gets a blank view controller
                viewController = eu::nifti::ocu::view::ViewControllerFactory::createViewController(eu::nifti::ocu::VIEW_TYPE_UNINITIALIZED, NULL, NULL, NULL, NULL);
                viewController->activate();
            }

            double VisualizationManagerForBlankScreen::getFieldOfViewHorizontal() const
            {
                return 0;
            }

            double VisualizationManagerForBlankScreen::getFieldOfViewVertical() const
            {
                return 0;
            }

            const std::string& VisualizationManagerForBlankScreen::getFixedFrame() const
            {
                return s;
            }

            const std::string& VisualizationManagerForBlankScreen::getTargetFrame() const
            {
                return s;
            }

            void VisualizationManagerForBlankScreen::resetTime()
            {

            }

            Ogre::Viewport* VisualizationManagerForBlankScreen::getViewport() const
            {
                return renderPanel->getViewport();
            }

            eu::nifti::ocu::view::IViewController* VisualizationManagerForBlankScreen::getViewController() const
            {
                return viewController;
            }

            void VisualizationManagerForBlankScreen::lockRender()
            {

            }

            void VisualizationManagerForBlankScreen::unlockRender()
            {

            }

            void VisualizationManagerForBlankScreen::queueRender()
            {

            }

            void VisualizationManagerForBlankScreen::forceReRender()
            {

            }

            void VisualizationManagerForBlankScreen::update(float wall_dt, float ros_dt)
            {
            }

            void VisualizationManagerForBlankScreen::onIdle(wxIdleEvent& evt)
            {

            }

            void VisualizationManagerForBlankScreen::lookAt(const Ogre::Vector3& point)
            {

            }

            void VisualizationManagerForBlankScreen::updateOCUInfoViewContent()
            {

            }

            void VisualizationManagerForBlankScreen::updateOCUInfoViewSize()
            {
                viewParams->viewWidth = getViewport()->getActualWidth();
                viewParams->viewHeight = getViewport()->getActualHeight();
            }

            void VisualizationManagerForBlankScreen::handleSizeEvent(wxSizeEvent& evt)
            {

            }

            const rviz::RenderPanel* VisualizationManagerForBlankScreen::getRenderPanel() const
            {
                return renderPanel;
            }

            Ogre::Vector3 VisualizationManagerForBlankScreen::getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const
            {
                return Ogre::Vector3();
            }

            eu::nifti::ocu::ViewType VisualizationManagerForBlankScreen::getViewType() const
            {
                return eu::nifti::ocu::VIEW_TYPE_UNINITIALIZED;
            }

            void VisualizationManagerForBlankScreen::setViewType(ViewType viewType)
            {

            }

            void VisualizationManagerForBlankScreen::changeViewParams(nifti_ocu_msgs::NIFTiViewParams* viewParams)
            {
                this->viewParams = viewParams;
            }

            // Todo: implement

            Ogre::Image VisualizationManagerForBlankScreen::getCurrentImage() const
            {
                throw "Not yet implemented";
            }

            std::string VisualizationManagerForBlankScreen::getCurrentImageEncoding() const
            {
                throw "Not yet implemented";
            }

            std::string VisualizationManagerForBlankScreen::toString() const
            {
                using namespace std;

                stringstream ss;


                ss << "View Type: " << getViewType() << " (eu::nifti::ocu::VIEW_TYPE_UNINITIALIZED)" << endl;
                ss << "Render Panel #" << renderPanel->getPanelNumber() << ": " << &renderPanel << " (" << renderPanel->GetSize().GetWidth() << " x " << renderPanel->GetSize().GetHeight() << ")" << endl;
                ss << "Render Window: " << renderPanel->getRenderWindow()->getWidth() << " x " << renderPanel->getRenderWindow()->getHeight() << endl;
                ss << "Render Viewport: " << renderPanel->getViewport()->getActualWidth() << " x " << renderPanel->getViewport()->getActualHeight() << endl;
                ss << "Render Viewport Overlays Enabled: " << renderPanel->getViewport()->getOverlaysEnabled() << endl;
                ss << "Render Viewport ClearEveryFrame: " << renderPanel->getViewport()->getClearEveryFrame() << endl;
                ss << "Render Window Active: " << renderPanel->getRenderWindow()->isActive() << endl;
                ss << "Render Window AutoUpdated: " << renderPanel->getRenderWindow()->isAutoUpdated() << endl;


                return ss.str();
            }





        }
    }
}
