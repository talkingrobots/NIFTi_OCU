// Benoit 2011-09-14
// Re-done 2013-01-29

#ifndef EU_NIFTI_OCU_GUI_VIEW_TYPE_FILMSTRIP_PANEL_FOR_VIEW_TYPES_H_
#define EU_NIFTI_OCU_GUI_VIEW_TYPE_FILMSTRIP_PANEL_FOR_VIEW_TYPES_H_

#include "Panels/FilmstripPanel.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IMultiVisualizationManager;
            
            namespace gui
            {

                /**
                 * Panel that allows choosing view types (camera or map)
                 */
                class FilmstripPanelForViewTypes : public FilmstripPanel
                {
                public:
                    FilmstripPanelForViewTypes(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr);
                    virtual ~FilmstripPanelForViewTypes();
                    
                    wxString getItemText(const u_int index) const;

                    void addViewType(const wxWindowID buttonID, const std::string& type, const std::string& name, const std::string& toolTipText);
                    
                protected:
                    void insertNewFrame(const FrameToAdd& newFrame);
                    
                    void onMouseLeftDown(wxMouseEvent& event);
                    
                    IMultiVisualizationManager* multiVizMgr;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_GUI_BOTTOM_PANEL_H_ */