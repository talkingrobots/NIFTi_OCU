// Benoit 2011-05-27

#ifndef EU_NIFTI_OCU_SELECTION_EOI_SELECTION_MANAGER_H
#define EU_NIFTI_OCU_SELECTION_EOI_SELECTION_MANAGER_H

#include <ros/publisher.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include <eu_nifti_cast/WorkingMemoryPointer.h>

#include "Selection/Common.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                class IUSARMarker;
                class LocationOfInterestMarker;
                class ObjectOfInterestMarker;
            }

            class IMultiVisualizationManager;
            class IVisualizationManager;

            namespace selection
            {
                class MarkerSelectionManager;
                class LocationSelectionManager;
                
                /**
                 * The selection manager is the overall class that others should use. It contains an object selection
                 * manager and a location selection manager. When a click is issued, it first tries to select an object,
                 * and if nothing is found, it tries to select a location.
                 * @param multiVizMgr
                 */
                class SelectionManager
                {
                public:
                    
                    enum SelectionType { SELECTION_TYPE_NONE, SELECTION_TYPE_OBJECT_OF_INTEREST, SELECTION_TYPE_LOCATION_OF_INTEREST, SELECTION_TYPE_ANNOTATED_PICTURE };

                    SelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr);
                    ~SelectionManager();

                    SelectionManager::SelectionType getSelectionType() const;
                    
                    void initialize();

                    /**
                     * Adds an object to be tracked for selection
                     * @param ooiMarker
                     * @return 
                     */
                    rviz::CollObjectHandle addTrackedMarker(eu::nifti::ocu::display::IUSARMarker* marker);
                    void removeTrackedMarker(const rviz::CollObjectHandle obj);

                    /**
                     * First tries to select an object of interest at the given pixel, within a square that extends
                     * from the pixel and goes "range" pixels to the left, right, top, and bottom. If no object is
                     * found, then tries to select a location. If the sky was clicked, then nothing is selected.
                     * @param vizMgr
                     * @param x
                     * @param y
                     * @param range
                     */
                    void select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y, int range = 0);
                    
                    /**
                     * 
                     * @param marker
                     */
                    void selectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                    void deselect();

                    const eu::nifti::ocu::display::IUSARMarker* getSelectedMarker();

                    /**
                     * Makes the camera(s) turn towards the selection
                     */
                    void focusOnSelection();

                protected:
                    void publishSelectionMessage(int uuid, int action);
                    void publishSelectionMessage(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, int action);
                    void publishSelectionMessage(const std::string& filename, int action);
                    
                    eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr;
                    
                    LocationSelectionManager* locationSelMgr;
                    MarkerSelectionManager* markerSelMgr;
                    
                    const EXIFReader_msgs::AnnotatedPicture* selectedPicture;
                    
                    SelectionType selectionType;
                    
                    ros::Publisher publisherEOISelection;
                    ros::Publisher publisherPictureSelection;
                    
                };
            }

        }
    }
}

#endif // EU_NIFTI_OCU_SELECTION_EOI_SELECTION_MANAGER_H
