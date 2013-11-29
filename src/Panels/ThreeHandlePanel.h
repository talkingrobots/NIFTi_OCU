// Benoit 2013-01-25

#ifndef EU_NIFTI_OCU_GUI_THREE_HANDLE_PANEL_H
#define EU_NIFTI_OCU_GUI_THREE_HANDLE_PANEL_H

#include <wx/panel.h>
#include <wx/sizer.h>

#include "IPullHandleEventHandler.h"

class wxAuiManager;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            
            namespace gui
            {
                class PullHandle;

                /**
                 * Panel that goes at the bottom of the screen and contains three "handles" to open up three panels
                 */
                class ThreeHandlePanel : public wxPanel, IPullHandleEventHandler
                {
                public:

                    ThreeHandlePanel(wxWindow *parentWindow, wxAuiManager* auiManager);
                    
                    void setPanels(wxPanel *panel1, wxPanel *panel2, wxPanel *panel3);
                    
                    void openPanel(u_int panelNumber);
                    void closePanel();
                    
                    
                    
                protected:

                    void onPullHandleClosed(PullHandle* handle);
                    void onPullHandleOpened(PullHandle* handle);
                    void onPullHandleClicked(PullHandle* handle);
                    
                    void openPanel(wxPanel *panelToOpen);
                    
                    void adjustLayout();
                    
                    wxAuiManager* auiManager;
                    
                    PullHandle *handle1, *handle2, *handle3;
                    wxPanel *panel1, *panel2, *panel3, *panelOpened;
                    
                    wxBoxSizer* sizerVertical;
                    
                    wxSize minSizeWithPanelsClosed;

                };

            }
        }
    }
}

#endif /* EU_NIFTI_PICVIEW_GUI_FILMSTRIP_PANEL_H */