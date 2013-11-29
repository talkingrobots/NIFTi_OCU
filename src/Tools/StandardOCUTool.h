// Benoit 2012-05-22

#ifndef EU_NIFTI_OCU_TOOLS_STANDARD_OCU_TOOL_H
#define EU_NIFTI_OCU_TOOLS_STANDARD_OCU_TOOL_H

#include <wx/timer.h>

#include "Tools/Tool.h"

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
            namespace view
            {
                class IViewController;
            }
            
            namespace tools
            {

                /**
                 * This tool is a template for the standard NIFTi OCU tools. It controls the view and allows for a special function on short clicks.
                 */
                class StandardOCUTool : public Tool, public wxEvtHandler
                {
                public:
                    StandardOCUTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl);

                    virtual void deactivate();

                    bool handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt);
                    bool handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);
                    
                    void onTimerLongClick(wxTimerEvent& event);

                protected:

                    virtual bool onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt) = 0;
                                       
                    const ros::Publisher* publisherViewControl;

                    wxTimer timerLongClick;
                    
                    bool dragging;
                    int xOnMouseDown;
                    int yOnMouseDown;
                    long timeOnMouseDown;
                    
                    eu::nifti::ocu::view::IViewController* currentViewController;

                    static const u_int DELAY_LONG_CLICK;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_TOOLS_STANDARD_OCU_TOOL_H

