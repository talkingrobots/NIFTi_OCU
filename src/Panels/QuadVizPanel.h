// Benoit Oct 2010

#ifndef NIFTI_QUAD_VIZ_PANEL_H_
#define NIFTI_QUAD_VIZ_PANEL_H_

#include <wx/panel.h>

#include "NIFTiViewsUtil.h"

class wxFlexGridSizer;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            namespace gui
            {

                /**
                 * This panel handles the placement of one to four visualization windows
                 * on the screen
                 * @param parentWindow
                 */
                class QuadVizPanel : public wxPanel
                {
                public:
                    QuadVizPanel(wxWindow *parentWindow);
                    void setMode(MultiVizMode mode, wxWindow* display0 = NULL, wxWindow* display1 = NULL, wxWindow* display2 = NULL, wxWindow* display3 = NULL);
                    
//                    /**
//                     * Replaces the display at the given index by the new given display
//                     */
//                    void replaceDisplay(wxWindow* oldDisplay, wxWindow* newDisplay);
                    
                    MultiVizMode getMode();
                protected:
                    wxFlexGridSizer* sizer;
                    MultiVizMode mode;
                };

            }
        }
    }
}

#endif /* NIFTI_QUAD_VIZ_PANEL_H_ */