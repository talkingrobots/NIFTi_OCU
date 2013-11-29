// Benoit 2012-06-29

#ifndef EU_NIFTI_PICVIEW_GUI_OCU_FRAME_H
#define EU_NIFTI_PICVIEW_GUI_OCU_FRAME_H

#include "Panels/VisualizationFrame.h"

class wxToolBar;

namespace ros
{
    class Publisher;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            class OCUMessagePublisherThread;
            class UUIDsManager;

            namespace gui
            {
                class SimplifiedDialoguePanel;
                class RightPanel;
                class FilmstripPanelForInFieldPics;
                class FilmstripPanelForViewTypes;
                class FilmstripPanel;
                class ThreeHandlePanel;
                class TopPanel;
            }

            namespace selection
            {
                class SelectionManager;
            }

        }
    }
}

namespace rviz
{

    /**
     * Main window of the software
     * @param parent
     */
    class OCUFrame : public VisualizationFrame
    {
    public:
        OCUFrame();
        ~OCUFrame();

        void OnSize(wxSizeEvent& event);

        // From IMultiVisualizationManager
        void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt, int panelNumber);
        void handleSizeEvent(wxSizeEvent& evt, int pnlNumber);
        void handleDragAndDropEvent(const wxString& text, int panelNum);

        eu::nifti::ocu::ViewType getViewType(u_int vizPanel);
        void setViewType(eu::nifti::ocu::ViewType viewType, u_int vizPanel);

        eu::nifti::ocu::MultiVizMode getVizMode() const;
        void setVizMode(eu::nifti::ocu::MultiVizMode mode);

        void hideElementOfInterestPanel();
        
        void onSelectObjectOfInterest(const eu_nifti_env::ObjectOfInterest* ooi, bool selected);

        void onSelectLocationOfInterest(const eu_nifti_env::LocationOfInterest* loi, bool selected);

        void onSelectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture, bool selected);

    protected:

        void addTool(eu::nifti::ocu::tools::Tool* tool, bool setActive = false);

        void setActiveTool(eu::nifti::ocu::tools::Tool* tool);

        void preInit();
        void createRenderPanels();
        void createSceneManagers();
        void createVizualizationManagers();
        void createPanels();
        void createTools();
        void setDefaultViewTypes();
        void postInit();

        void onUpdate();
        void updateVizModeAndViews();

        void customizeFilmstripPanelForViewTypes(eu::nifti::ocu::gui::FilmstripPanelForViewTypes* pnlMaps, eu::nifti::ocu::gui::FilmstripPanelForViewTypes* pnlCameras);

        // wx Callbacks

        void onToolClicked(wxCommandEvent& event);

        void setViewTypeSynchronized(eu::nifti::ocu::ViewType viewType, u_int vizPanel);
        void setVizModeSynchronized(eu::nifti::ocu::MultiVizMode mode);


        void updateOCUinfoViewSize(int panelNum);
        void updateOCUinfoViewType(int panelNum);

        void adjustMouseCursors();

        void forceRightPanelToExpand();

        eu::nifti::ocu::gui::TopPanel* topPanel;
        eu::nifti::ocu::gui::SimplifiedDialoguePanel* dialoguePanel;
        eu::nifti::ocu::gui::RightPanel* rightPanel;
        eu::nifti::ocu::gui::ThreeHandlePanel* threeHandlePanel;
        //eu::nifti::ocu::gui::UAVControlPanel* uavControlPanel; // Temporarily disabled until the UAV is stable enough
        eu::nifti::ocu::gui::FilmstripPanelForInFieldPics* pnlInFieldPics;
        eu::nifti::ocu::gui::FilmstripPanel* pnlFilmstrips[3];

        wxToolBar* toolBarForTools;
        wxToolBar* toolBarCameras;
        wxToolBar* toolBarMaps;

        eu::nifti::ocu::selection::SelectionManager* selectionMgr;

        bool viewTypeWasRecentlySelected;
        eu::nifti::ocu::ViewType recentlySelectedViewType;
        uint32_t recentlySelectedViewTypePanelNumber;
        const EXIFReader_msgs::AnnotatedPicture* recentlySelectedAnnotatedPicture;

        bool vizModeWasRecentlySelected;
        eu::nifti::ocu::MultiVizMode recentlySelectedMultiVizMode;

        eu::nifti::ocu::OCUMessagePublisherThread* ocuMsgPubThread;
        eu::nifti::ocu::UUIDsManager* uuidMgr;

        ros::Publisher publisherToolActivation;
        ros::Publisher publisherViewControl;

        bool draggingInProgress;

        wxCursor cursorCamera, cursorGoTo;

        static const char* TOPIC_TOOL_ACTIVATION;
        static const char* TOPIC_VIEW_CONTROL;

        static const int WINDOW_WIDTH;
        static const int WINDOW_HEIGHT;

    private:

        /**
         * Moves the a view to another position in our list when changing the total number of views
         * @param indexFrom
         * @param indexTo
         */
        void swapView(uint32_t indexFrom, uint32_t indexTo);

        /**
         * Removes the selected views (panel & viz mgr)
         * @param index
         */
        void removeView(uint32_t index);

    };

}

#endif // EU_NIFTI_PICVIEW_GUI_OCU_FRAME_H
