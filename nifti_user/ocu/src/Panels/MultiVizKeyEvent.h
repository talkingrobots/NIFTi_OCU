// Benoit 2008-08-30

#ifndef EU_NIFTI_OCU_GUI_MULTI_VIZ_KEY_EVENT_H
#define EU_NIFTI_OCU_GUI_MULTI_VIZ_KEY_EVENT_H

#include <wx/event.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IVisualizationManager;
            
            namespace gui
            {

                struct MultiVizKeyEvent
                {

                    MultiVizKeyEvent(wxKeyEvent& evt, eu::nifti::ocu::IVisualizationManager * vizMgr)
                    : evt(evt)
                    , vizMgr(vizMgr)
                    {
                    }

                    wxKeyEvent evt;
                    eu::nifti::ocu::IVisualizationManager* vizMgr;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_GUI_MULTI_VIZ_KEY_EVENT_H
