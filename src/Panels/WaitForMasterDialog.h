// Benoit September 2010
// This class was generated from something, but I don't know what.

#include <wx/dialog.h>

#include "Panels/WaitForMasterThread.h"

class wxStaticText;
class wxButton;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                
                class WaitForMasterDialog : public wxDialog
                {
                public:

                    WaitForMasterDialog(const std::string& rosMasterURI);
                    ~WaitForMasterDialog();

                    void rosChecked();

                protected:
                    wxStaticText* text_;
                    wxButton* cancel_button_;
                    
                    void onClose(wxCloseEvent& event);

                    void onCancel(wxCommandEvent& event);

                };

            }
        }
    }
}