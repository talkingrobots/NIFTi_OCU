// Benoit 2011-05-27

#ifndef EU_NIFTI_OCU_SELECTION_LOI_SELECTION_MANAGER_H
#define EU_NIFTI_OCU_SELECTION_LOI_SELECTION_MANAGER_H

#include <ros/publisher.h>

#include <eu_nifti_env/LocationOfInterest.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                class LocationOfInterestMarker;
            }

            class IMultiVisualizationManager;
            class IVisualizationManager;

            namespace selection
            {

                /**
                 * Handles the selection of locations, by showing a green cone 
                 * marker and publishing the appropriate messages
                 * @param multiVizMgr
                 */
                class LocationSelectionManager
                {
                public:

                    LocationSelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr);
                    ~LocationSelectionManager();

                    const eu::nifti::ocu::display::LocationOfInterestMarker* select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y);
                    void deselect();

                    const eu::nifti::ocu::display::LocationOfInterestMarker* getSelectedMarker();

                    void focusOnSelection();

                protected:
                    
                    /**
                     * Publishes a ROS message about the creation of a new LOI
                     */
                    void publishCreationMessage();

                    ros::Publisher publisherLOICreation;
                    
                    eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr;
                    
                    eu::nifti::ocu::display::LocationOfInterestMarker* selection;

                    static const char* TOPIC_EOI_CREATION;
                };
            }

        }
    }
}

#endif // EU_NIFTI_OCU_SELECTION_LOI_SELECTION_MANAGER_H
