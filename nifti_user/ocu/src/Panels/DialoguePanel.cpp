// Benoit 2010-12-02

#include <wx/sizer.h>
#include <wx/textctrl.h>
#include <wx/stattext.h>
#include <wx/scrolwin.h>
#include <wx/thread.h> 

#include "NIFTiROSUtil.h"

#include "DialoguePanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                const char* DialoguePanel::DIALOGUE_TOPIC = "ocu/dialogue";

                const u_int DialoguePanel::MAX_LENGTH_HISTORY = 2000; // If the chat history becomes longer than this, then it will be cut to keep only the most recent history

                DialoguePanel::DialoguePanel(wxWindow *parentWindow)
                : wxPanel(parentWindow)
                {
                    //this->SetBackgroundColour(wxColour(0, 255, 0)); // Makes the background green so that it's easy to see

                    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

                    scWindow = new wxScrolledWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
                    //scWindow->SetScrollRate(0,5); //Maybe useless
                    scWindow->SetScrollbars(0, 20, 0, 50);

                    lblHistory = new wxStaticText(scWindow, wxID_ANY, wxT("CHAT HISTORY"), wxDefaultPosition, wxSize(650, -1), 0);
                    //lblHistory = new wxStaticText(scWindow, wxID_ANY, wxT("wxTextCtrl text format The multiline text controls always store the text as a sequence of lines separated by \n characters, i.e. in the Unix text format even on non-Unix platforms. This allows the user code to ignore the differences between the platforms but at a price: the indices in the control such as those returned by GetInsertionPoint or GetSelection can not be used as indices into the string returned by GetValue as they're going to be slightly off for platforms using \r\n as separator (as Windows does), for example.Instead, if you need to obtain a substring between the 2 indices obtained from the control with the help of the functions mentioned above, you should use GetRange. And the indices themselves can only be passed to other methods, for example SetInsertionPoint or SetSelection.To summarize: never use the indices returned by (multiline) wxTextCtrl as indices into the string it contains, but only as arguments to be passed back to the other wxTextCtrl methods."), wxDefaultPosition, wxSize(650,-1),0);
                    lblHistory->Wrap(650);

                    txtInput = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(650, -1), wxTE_PROCESS_ENTER);
                    txtInput->SetMaxLength(128);

                    sizer->Add(txtInput, 0, wxALL, 5);
                    sizer->Add(scWindow, 1, wxALL, 5);

                    this->SetSizer(sizer);
                    this->Layout();

                    txtInput->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler(DialoguePanel::onEnterPressed), NULL, this);

                    publisher = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<eu_nifti_ocu_msg_ros::DialogueUtteranceMessage > (DIALOGUE_TOPIC, 1);
                    subscriber = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->subscribe(DIALOGUE_TOPIC, 10, &DialoguePanel::onDialogueUtteranceMessageReceived, this);
                }

                DialoguePanel::~DialoguePanel()
                {
                }

                void DialoguePanel::onDialogueUtteranceMessageReceived(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessageConstPtr& msg)
                {
                    //std::cout << "onDialogueUtteranceMessageReceived: " << msg->utterance << std::endl;

                    // Since I changed the ICE connection to a ROS connection (1 June 2011), I cannot acquire the mutex
                    //wxMutexGuiEnter();
                    
                    if (lblHistory->GetLabel().Length() > MAX_LENGTH_HISTORY)
                    {
                        lblHistory->SetLabel(lblHistory->GetLabel().Left(MAX_LENGTH_HISTORY / 2));
                    }
                    
                    if(msg->userID.length() == 0) // Means that it comes from the robot (and not a user)
                    {
                        lblHistory->SetLabel(wxString("Robot: ", wxConvUTF8).Append(wxString(msg->utterance.c_str(), wxConvUTF8)).Append(wxT("\n")).Append(lblHistory->GetLabel()));
                    }
                    else
                    {
                        lblHistory->SetLabel(wxString("User: ", wxConvUTF8).Append(wxString(msg->utterance.c_str(), wxConvUTF8)).Append(wxT("\n")).Append(lblHistory->GetLabel()));
                    }
                    
                    lblHistory->Wrap(scWindow->GetSize().GetWidth() - 20); // Todo make constant

                    //wxMutexGuiLeave();

                    //std::cout << "OUT onDialogueUtteranceMessageReceived: " << msg->utterance << std::endl;
                }

                void DialoguePanel::onEnterPressed(wxCommandEvent& evt)
                {
                    //ROS_INFO("IN void DialoguePanel::onEnterPressed(wxCommandEvent& evt)");
                    
                    // Since I changed the ICE connection to a ROS connection, I cannot acquire the mutex
                    //wxMutexGuiEnter();
                    
                    // Does nothing if there is no text to send
                    if (txtInput->GetValue() == wxEmptyString) return;

                    eu_nifti_ocu_msg_ros::DialogueUtteranceMessage msg;
                    msg.header.stamp = ros::Time::now();
                    msg.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msg.utterance = txtInput->GetValue().mb_str();
                    publisher.publish(msg);

                    // Clears the input field
                    txtInput->SetValue(wxEmptyString);
                    
                    //wxMutexGuiLeave();
                    
                    //ROS_INFO("OUT void DialoguePanel::onEnterPressed(wxCommandEvent& evt)");
                }

            }
        }
    }
}