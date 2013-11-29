//Benoit August 2010

#ifndef NIFTI_I_MULTI_VISUALIZATION_INPUT_EVENTS_HANDLER_H
#define NIFTI_I_MULTI_VISUALIZATION_INPUT_EVENTS_HANDLER_H

class wxSizeEvent;
class wxString;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                class MultiVizKeyEvent;
                class MultiVizMouseEvent;

                /**
                 * This interface defines an object that can handle events generated from
                 * the user via the mouse or keyboard
                 */
                class IMultiVizInputEventsHandler
                {
                public:
                    virtual void handleKeyEvent(MultiVizKeyEvent& evt, int pnlNumber) = 0;
                    virtual void handleMouseEvent(MultiVizMouseEvent& evt, int pnlNumber) = 0;
                    virtual void handleSizeEvent(wxSizeEvent& evt, int pnlNumber) = 0;
                    virtual void handleDragAndDropEvent(const wxString& text, int panelNum) = 0;
                };

            }
        }
    }
}

#endif //NIFTI_I_MULTI_VISUALIZATION_INPUT_EVENTS_HANDLER_H
