// Benoit 2010-12-02

// Todo Benoit Comment

#ifndef NIFTI_PLANNING_PANEL_H_
#define NIFTI_PLANNING_PANEL_H_

#include <wx/panel.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include "eclipse_prolog_msgs/ActionScheduled.h"

class wxGridSizer;
class wxStaticText;
class wxButton;
class wxScrolledWindow;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                class PlanningPanel : public wxPanel
                {
                public:
                    PlanningPanel(wxWindow *parentWindow);
                    ~PlanningPanel();

                    void onMsgReceived(const eclipse_prolog_msgs::ActionScheduledConstPtr& msg); // When I receive a new update from the robot

                protected:
                    void onClick(wxCommandEvent& evt);

                    /**
                     * Disables this DialoguePanel so that the user cannot use it anymore
                     */
                    void disablePanelDueToLostConnection();

                    //wxBoxSizer* sizer;
                    wxScrolledWindow* scWindow;
                    wxStaticText* lblPlan;
                    wxButton* cmdPauseResume;

                    ros::NodeHandle nh;
                    ros::Subscriber subscriber;

                    static const char* ROS_TOPIC;
                    
                    static const int PLAN_WIDTH;
                    static const int PLAN_HEIGHT;
                };

            }
        }
    }
}

#endif /* NIFTI_PLANNING_PANEL_H_ */