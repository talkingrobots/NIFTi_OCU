//Benoit September 2010

#ifndef NIFTI_I_MULTI_VISUALIZATION_MANAGER_H
#define NIFTI_I_MULTI_VISUALIZATION_MANAGER_H

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include <eu_nifti_env/LocationOfInterest.h>
#include <eu_nifti_env/ObjectOfInterest.h>

#include "IMultiVizInputEventsHandler.h"
#include "NIFTiViewsUtil.h"

namespace Ogre
{
    class SceneManager;
    class Vector3;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                class IUSARMarker;
            }

            /**
             * Manages a scene with multiple views, and thus multiple
             * visualization managers
             */
            class IMultiVisualizationManager : public eu::nifti::ocu::gui::IMultiVizInputEventsHandler
            {
            public:

                /////////////////////////////////////////////
                // From IMultiVisualizationInputEventsHandler
                // virtual void handleKeyEvent(MultiVizKeyEvent& evt, int pnlNumber) = 0;
                // virtual void handleMouseEvent(MultiVizMouseEvent& evt, int pnlNumber) = 0;
                // virtual void handleSizeEvent(wxSizeEvent& evt, int pnlNumber) = 0;
                // virtual void handleDragAndDropEvent(const wxString& text, int panelNum) = 0;
                /////////////////////////////////////////////

                /**
                 * Gets the Ogre scene manager
                 */
                virtual Ogre::SceneManager* getSceneManager() const = 0;

                /**
                 * Ensures that Ogre's renderer is called only in a mutex block
                 */
                virtual void lockRender() = 0;

                /**
                 * Ensures that Ogre's renderer is called only in a mutex block
                 */
                virtual void unlockRender() = 0;

                /**
                 * Queues a render.  Multiple calls should cause only a single render
                 * This function should be callable from any thread.
                 */
                virtual void queueRender() = 0;

                /**
                 * Makes all views point to a specific location
                 */
                virtual void lookAt(const Ogre::Vector3& point) = 0;

                /**
                 * Gets the view type that is displayed in the given visualization panel
                 */
                virtual eu::nifti::ocu::ViewType getViewType(u_int vizPanel) = 0;

                /**
                 * Sets a particular view type in one of the visualization panels
                 */
                virtual void setViewType(eu::nifti::ocu::ViewType viewType, u_int vizPanel) = 0;

                /**
                 * Gets the visualization mode (single view, dual vertical, dual horizontal, quad)
                 */
                virtual eu::nifti::ocu::MultiVizMode getVizMode() const = 0;

                /**
                 * Sets the visualization mode (single view, dual vertical, dual horizontal, quad)
                 */
                virtual void setVizMode(eu::nifti::ocu::MultiVizMode mode) = 0;

                /**
                 * Hides the ElementOfInterest panel
                 */
                virtual void hideElementOfInterestPanel() = 0;

                /**
                 * Performs appropriate actions depending on the object that got (de)selected
                 */
                virtual void onSelectObjectOfInterest(const eu_nifti_env::ObjectOfInterest* ooi, bool selected) = 0;

                /**
                 * Performs appropriate actions depending on the object that got (de)selected
                 */
                virtual void onSelectLocationOfInterest(const eu_nifti_env::LocationOfInterest* loi, bool selected) = 0;

                /**
                 * Performs appropriate actions depending on the object that got (de)selected
                 */
                virtual void onSelectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture, bool selected) = 0;

            };

        }
    }
}

#endif // NIFTI_I_MULTI_VISUALIZATION_MANAGER_H
