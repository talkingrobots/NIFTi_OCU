// Benoit 2013-01-28

#ifndef EU_NIFTI_OCU_GUI_PULL_HANDLE_H
#define EU_NIFTI_OCU_GUI_PULL_HANDLE_H

#include <wx/bitmap.h>
#include <wx/panel.h>

class wxStaticBitmap;

namespace eu
{

    namespace nifti
    {

        namespace ocu
        {

            namespace gui
            {

                class IPullHandleEventHandler;

                /**
                 * Tries to mimic a handle that can be dragged, for example to show/hide panels. The user must manually
                 * tell the handle when it can and cannot open or close. This allows it to work in groups of handles.
                 */
                class PullHandle : public wxPanel
                {
                public:
                    
                    enum OpeningDirection
                    {
                        UP, DOWN, LEFT, RIGHT
                    };

                    PullHandle(wxWindow* parent, IPullHandleEventHandler *handler, const std::string& imagePathClosed, const std::string& imagePathOpened, OpeningDirection openingDirection, bool startOpened);

                    bool getOpeneability() const;
                    void setOpeneability(bool canOpen);
                    
                    bool getCloseability() const;
                    void setCloseability(bool canClose);
                    
                protected:

                    void onMouseDown(wxMouseEvent& event);
                    void onMouseUp(wxMouseEvent& event);
                    void onMouseLeave(wxMouseEvent& event);
                    
                    void onPullHandleDraggedToOpen();
                    void onPullHandleDraggedToClose();

                    wxBitmap bmpClosed, bmpOpened;
                    wxStaticBitmap* staticBitmap;

                    IPullHandleEventHandler *handler;

                    int xOnMouseDown, yOnMouseDown;
                    bool draggingHandle;
                    
                    const OpeningDirection OPENING_DIRECTION;
                    
                    bool canOpen, canClose;
                };

            }
        }
    }
}



#endif // EU_NIFTI_OCU_GUI_PULL_HANDLE_H
