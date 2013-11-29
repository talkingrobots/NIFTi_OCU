// Benoit September 2010
// This class was generated from something, but I don't know what. (probably and IDE to make wxWidgets UI)

#include <sstream>

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include "WaitForMasterDialog.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                WaitForMasterDialog::WaitForMasterDialog(const std::string& rosMasterURI)
                : wxDialog(NULL, wxID_ANY, wxT("Waiting for ROS Master"))
                {
                    // The size is hard-coded because wx creates it too big otherwise
                    this->SetSize(350, 75);

                    wxBoxSizer* bSizer15;
                    bSizer15 = new wxBoxSizer(wxVERTICAL);

                    std::stringstream ss;
                    ss << "Contacting ROS master at [" << rosMasterURI << "]....";

                    text_ = new wxStaticText(this, wxID_ANY, wxString::FromAscii(ss.str().c_str()), wxDefaultPosition, wxDefaultSize, 0);
                    text_->Wrap(-1);
                    bSizer15->Add(text_, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 5);

                    cancel_button_ = new wxButton(this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0);
                    cancel_button_->SetDefault();
                    bSizer15->Add(cancel_button_, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 5);

                    this->SetSizer(bSizer15);
                    this->Layout();

                    this->Centre(wxBOTH);

                    // Connect Events
                    this->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(WaitForMasterDialog::onClose));
                    cancel_button_->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(WaitForMasterDialog::onCancel), NULL, this);
                }

                WaitForMasterDialog::~WaitForMasterDialog()
                {
                    // Disconnect Events
                    this->Disconnect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(WaitForMasterDialog::onClose));
                    cancel_button_->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(WaitForMasterDialog::onCancel), NULL, this);
                }

                void WaitForMasterDialog::onClose(wxCloseEvent& event)
                {
                    EndModal(wxID_CANCEL);
                }

                void WaitForMasterDialog::onCancel(wxCommandEvent& event)
                {
                    EndModal(wxID_CANCEL);
                }
                
                void WaitForMasterDialog::rosChecked()
                {
                    EndModal(wxID_OK);
                }

            }
        }
    }
}