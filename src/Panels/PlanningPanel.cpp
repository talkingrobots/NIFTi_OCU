// Benoit 2010-12-02

#include <sstream>

#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/statbox.h>
#include <wx/button.h>

#include "PlanningPanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const char* PlanningPanel::ROS_TOPIC = "/planner/task";

                const int PlanningPanel::PLAN_WIDTH = 240;
                const int PlanningPanel::PLAN_HEIGHT = 80; // This is a temporary value, because wxWidgets does not give any decent size to the label unless I fix it

                PlanningPanel::PlanningPanel(wxWindow *parentWindow)
                : wxPanel(parentWindow)
                {
                    //this->SetBackgroundColour(wxColour(0, 0, 255)); // Benoit Todo Remove this TEMP

                    // wxStaticBox *box = new wxStaticBox(panel, wxID_ANY, "StaticBox");
                    //new wxStaticText(box, wxID_ANY "This window is a child of the staticbox");

                    //wxStaticBox *box = new wxStaticBox(this, wxID_ANY, wxEmptyString);
                    //new wxStaticText(panel, wxID_ANY "This window is a child of the panel");

                    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

                    //                    scWindow = new wxScrolledWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
                    //                    //scWindow->SetScrollRate(0,5); //Maybe useless
                    //                    scWindow->SetScrollbars(0, 20, 0, 50);
                    //                    sizer->Add(scWindow, 0, wxALL, 0);

                    //                    lblPlan = new wxStaticText(scWindow, wxID_ANY, wxT("No autonomous plan received yet"), wxDefaultPosition, wxSize(PLAN_WIDTH, PLAN_HEIGHT), 0);
                    lblPlan = new wxStaticText(this, wxID_ANY, wxT("No autonomous plan received yet"), wxDefaultPosition, wxSize(PLAN_WIDTH, PLAN_HEIGHT), 0);
                    lblPlan->Wrap(PLAN_WIDTH);
                    sizer->Add(lblPlan, 0, wxALL, 0);

                    //cmdPauseResume = new wxButton(this, wxID_ANY, wxT("Pause"), wxDefaultPosition, wxDefaultSize, 0);
                    //sizer->Add(cmdPauseResume, 0, wxALL, 5);

                    sizer->SetSizeHints(this); // set size hints to honour minimum size
                    this->SetSizer(sizer);
                    this->Layout();

                    //cmdPauseResume->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(PlanningPanel::onClick), NULL, this);


                    // Subscribes to the execution monitoring node
                    subscriber = nh.subscribe(ROS_TOPIC, 1, &PlanningPanel::onMsgReceived, this);

                }

                PlanningPanel::~PlanningPanel()
                {

                }

                void PlanningPanel::onMsgReceived(const eclipse_prolog_msgs::ActionScheduledConstPtr& msg)
                {
                    //        cout << "TASK: " << msg->task_name << endl;
                    //        cout << "ACTION: " << msg->action << endl;
                    //        cout << "STATUS: " << msg->status << endl;

                    ostringstream formattedPlan;
                    formattedPlan << "TASK: " << msg->task_name << endl;
                    formattedPlan << "ACTION: " << msg->action << endl;
                    formattedPlan << "STATUS: " << msg->status << endl;

                    //wxMutexGuiEnter();    // This should be there, but somehow it makes the GUI freeze

                    lblPlan->SetLabel(wxString(formattedPlan.str().c_str(), wxConvUTF8));
                    lblPlan->Wrap(PLAN_WIDTH);

                    //wxMutexGuiLeave();    // This should be there, but somehow it makes the GUI freeze

                }

                void PlanningPanel::onClick(wxCommandEvent& evt)
                {


                }


            }
        }
    }
}