// Benoit 2010-12-02

#ifndef NIFTI_DIALOGUE_PANEL_H_
#define NIFTI_DIALOGUE_PANEL_H_

#include <wx/panel.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <eu_nifti_ocu_msg_ros/DialogueUtteranceMessage.h>

#include "NIFTiConstants.h"

class wxGridSizer;
class wxScrolledWindow;
class wxStaticText;
class wxTextCtrl;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                /**
                 * Displays a history of the dialogue between the robot and the user, and also allows to send text to the robot
                 * @param parentWindow
                 */
                class DialoguePanel : public wxPanel
                {
                public:
                    DialoguePanel(wxWindow *parentWindow);
                    ~DialoguePanel();

                    void onDialogueUtteranceMessageReceived(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessageConstPtr& msg); // When I receive a text message from the robot

                protected:
                    void onEnterPressed(wxCommandEvent& evt);

                    //wxBoxSizer* sizer;
                    wxStaticText* lblHistory;
                    wxTextCtrl* txtInput;

                    wxScrolledWindow* scWindow;

                    ros::Publisher publisher;
                    ros::Subscriber subscriber;
                    
                    static const char* DIALOGUE_TOPIC;

                    static const u_int MAX_LENGTH_HISTORY;
                };

            }
        }
    }
}

#endif /* NIFTI_DIALOGUE_PANEL_H_ */