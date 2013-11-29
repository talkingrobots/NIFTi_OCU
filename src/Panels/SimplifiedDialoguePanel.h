// Benoit 2011-12-05

#ifndef EU_NIFTI_OCU_GUI_SIMPLIFIED_DIALOGUE_PANEL_H_
#define EU_NIFTI_OCU_GUI_SIMPLIFIED_DIALOGUE_PANEL_H_

#include <wx/panel.h>
#include <wx/string.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <eu_nifti_ocu_msg_ros/DialogueUtteranceMessage.h>

#include "NIFTiConstants.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                /**
                 * Displays the last utterance received from the robot
                 */
                class SimplifiedDialoguePanel : public wxPanel
                {
                public:
                    SimplifiedDialoguePanel(wxWindow *parentWindow);

                    void onDialogueUtteranceMessageReceived(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessageConstPtr& msg);

                protected:
                    wxStaticText* lblHistory;
                    wxStaticText* lblLastUtterance;
                    wxString lastUtterance;

                    ros::Subscriber subscriber;
                    
                    static const char* DIALOGUE_TOPIC;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_GUI_SIMPLIFIED_DIALOGUE_PANEL_H_ */