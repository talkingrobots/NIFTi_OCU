// Benoit August 30, 2010

#ifndef EU_NIFTI_OCU_GUI_MULTI_VIZ_MOUSE_EVENT_H
#define EU_NIFTI_OCU_GUI_MULTI_VIZ_MOUSE_EVENT_H

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

                struct MultiVizMouseEvent
                {

                    MultiVizMouseEvent(wxMouseEvent& evt, int lastX, int lastY, eu::nifti::ocu::IVisualizationManager * vizMgr)
                    : evt(evt)
                    , lastX(lastX)
                    , lastY(lastY)
                    , vizMgr(vizMgr)
                    {
                    }

                    wxMouseEvent evt;
                    int lastX, lastY;
                    eu::nifti::ocu::IVisualizationManager* vizMgr;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_GUI_MULTI_VIZ_MOUSE_EVENT_H
