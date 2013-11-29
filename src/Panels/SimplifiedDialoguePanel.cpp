// Benoit 2011-12-05
// rostopic pub -r 1 /ocu/dialogue eu_nifti_ocu_msg_ros/DialogueUtteranceMessage '[0, 0, "/myFrame"]' "Robot" "Hello World"
// rostopic pub -r 2 /ocu/dialogue eu_nifti_ocu_msg_ros/DialogueUtteranceMessage '[0, '[1324046049,0]', "/myFrame"]' "User" "Spaghetti adjakd aksjdkasdj aksdj asd kadaskd jasd"

#include <wx/sizer.h>
#include <wx/stattext.h>

#include "NIFTiROSUtil.h"

#include "SimplifiedDialoguePanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                const char* SimplifiedDialoguePanel::DIALOGUE_TOPIC = "ocu/dialogue";

                const wxColor blueLikeBlueBotics(85, 142, 213);

                const int WIDTH = 700;

                SimplifiedDialoguePanel::SimplifiedDialoguePanel(wxWindow *parentWindow)
                : wxPanel(parentWindow)
                {
                    //this->SetBackgroundColour(wxColour(200, 255, 200)); // Makes the background green so that it's easy to see

                    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

                    lastUtterance = wxEmptyString;

                    lblHistory = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(WIDTH, -1), 0);
                    lblLastUtterance = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(WIDTH, -1), 0);
                    //lblLastUtterance->Wrap(WIDTH);

                    sizer->Add(lblHistory, 1, wxALL, 5);
                    sizer->Add(lblLastUtterance, 1, wxALL, 5);

                    this->SetSizer(sizer);
                    this->Layout();

                    subscriber = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->subscribe(DIALOGUE_TOPIC, 10, &SimplifiedDialoguePanel::onDialogueUtteranceMessageReceived, this);
                }

                void SimplifiedDialoguePanel::onDialogueUtteranceMessageReceived(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessageConstPtr& msg)
                {
                    //std::cout << "IN onDialogueUtteranceMessageReceived: " << msg->utterance << std::endl;                   

                    wxString formattedTime;
                    if (msg->header.stamp.sec != 0)
                    {
                        //std::cout << "header: " << msg->header.stamp.sec << " ros: " << ros::Time::now().sec << std::endl;
                        
                        time_t rawtime = msg->header.stamp.sec;
                        char timeCharBuffer[9];
                        strftime(timeCharBuffer, 9, "%H:%M:%S", localtime(&rawtime));
                        formattedTime = wxString(timeCharBuffer, wxConvUTF8);
                    }
                    else
                    {
                        formattedTime = wxT("[Unknown time]");
                    }

                    wxString userID;
                    if (msg->userID.length() != 0)
                    {
                        userID = wxString(msg->userID.c_str(), wxConvUTF8);
                    }
                    else
                    {
                        userID = wxT("[Unknown source]");
                    }

                    wxString newUtterance = formattedTime.Append(wxT(" - ")).Append(userID).Append(wxString(": \"", wxConvUTF8)).Append(wxString(msg->utterance.c_str(), wxConvUTF8)).Append(wxString("\"", wxConvUTF8));

                    //std::cout << lastUtterance.Append(wxString("\n", wxConvUTF8)).Append(newUtterance).mbc_str() << std::endl; 

                    lblHistory->SetLabel(lastUtterance);
                    lblLastUtterance->SetLabel(newUtterance);

                    //lblLastUtterance->Wrap(this->GetSize().GetWidth() - 20); // Todo make constant

                    // Adjusts the colors to make it blue for the robot

                    if (lastUtterance.Contains(wxT(" - Robot:")))
                    {
                        lblHistory->SetForegroundColour(blueLikeBlueBotics);
                    }
                    else
                    {
                        lblHistory->SetForegroundColour(wxColour(0, 0, 0));
                    }

                    if (newUtterance.Contains(wxT(" - Robot:")))
                    {
                        lblLastUtterance->SetForegroundColour(blueLikeBlueBotics);
                    }
                    else
                    {
                        lblLastUtterance->SetForegroundColour(wxColour(0, 0, 0));
                    }


                    lastUtterance = newUtterance;

                    //std::cout << lastUtterance.mb_str() << std::endl; 

                    //std::cout << "OUT onDialogueUtteranceMessageReceived: " << msg->utterance << std::endl;
                }

            }
        }
    }
}