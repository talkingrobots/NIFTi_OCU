// Benoit Oct 2010

#include <wx/sizer.h>

#include "QuadVizPanel.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                QuadVizPanel::QuadVizPanel(wxWindow *parentWindow)
                : wxPanel(parentWindow)
                {
                    //this->SetBackgroundColour(wxColour(0, 255, 0)); // Benoit Todo Remove this TEMP
                }

                MultiVizMode QuadVizPanel::getMode()
                {
                    return mode;
                }

                void QuadVizPanel::setMode(MultiVizMode mode, wxWindow* display0, wxWindow* display1, wxWindow* display2, wxWindow* display3)
                {
                    switch (mode)
                    {
                        case VIZ_MODE_SINGLE:

                            assert(display0 != NULL);

                            sizer = new wxFlexGridSizer(1, 1, 0, 0);
                            sizer->AddGrowableRow(0, 0);
                            sizer->AddGrowableCol(0, 0);
                            sizer->Add(display0, wxSizerFlags().Expand());

                            break;

                        case VIZ_MODE_DUAL_VERTICAL:

                            assert(display0 != NULL);
                            assert(display1 != NULL);

                            sizer = new wxFlexGridSizer(2, 1, 0, 0);
                            sizer->AddGrowableCol(0, 0);
                            sizer->AddGrowableRow(0, 0);
                            sizer->AddGrowableRow(1, 0);
                            sizer->Add(display0, wxSizerFlags().Expand());
                            sizer->Add(display1, wxSizerFlags().Expand());

                            break;

                        case VIZ_MODE_DUAL_HORIZONTAL:

                            assert(display0 != NULL);
                            assert(display1 != NULL);

                            sizer = new wxFlexGridSizer(1, 2, 0, 0);
                            sizer->AddGrowableRow(0, 0);
                            sizer->AddGrowableCol(0, 0);
                            sizer->AddGrowableCol(1, 0);
                            sizer->Add(display0, wxSizerFlags().Expand());
                            sizer->Add(display1, wxSizerFlags().Expand());

                            break;

                        case VIZ_MODE_QUAD:

                            assert(display0 != NULL);
                            assert(display1 != NULL);
                            assert(display2 != NULL);
                            assert(display3 != NULL);
                            
                            sizer = new wxFlexGridSizer(2, 2, 0, 0);
                            sizer->AddGrowableRow(0, 0);
                            sizer->AddGrowableRow(1, 0);
                            sizer->AddGrowableCol(0, 0);
                            sizer->AddGrowableCol(1, 0);
                            sizer->Add(display0, wxSizerFlags().Expand());
                            sizer->Add(display1, wxSizerFlags().Expand());
                            sizer->Add(display2, wxSizerFlags().Expand());
                            sizer->Add(display3, wxSizerFlags().Expand());

                            break;
                    }
                    
                    this->SetSizer(sizer, true); // Deletes the old sizer
                    this->mode = mode;

                    this->Layout();
                }
                
//                void QuadVizPanel::replaceDisplay(wxWindow* oldDisplay, wxWindow* newDisplay)
//                {
//                    sizer->Replace(oldDisplay, newDisplay);
//                    
//                    //this->Layout();
//                }

            }
        }
    }
}


// To remove elements
// Detach(i);
// Replace(i, newWindow);

