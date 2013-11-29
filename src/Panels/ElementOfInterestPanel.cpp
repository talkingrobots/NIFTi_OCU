// Benoit 2011-08-29
// rostopic pub -1 /flippers_cmd nifti_robot_driver_msgs/FlippersState -- -1.75 -1.75 1.75 1.75

#include <wx/button.h>
#include <wx/sizer.h>

#include <ros/node_handle.h>

#include <eu_nifti_env/ElementOfInterest.h>
#include <eu_nifti_env_msg_ros/ElementOfInterestConfirmationMessage.h>

#include "IMultiVisualizationManager.h"

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "ElementOfInterestPanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const char* ElementOfInterestPanel::CONFIRMATION_TOPIC = "/eoi_confirmation"; // CHANGED TEMPORARILY due to http://answers.ros.org/question/48086/problem-with-subscribing-to-a-topic-that-contains-slash/

                const int ElementOfInterestPanel::ID_BTN_CONFIRM = 0;
                const int ElementOfInterestPanel::ID_BTN_REJECT = 1;

                ElementOfInterestPanel::ElementOfInterestPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr, int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, ros::Publisher& publisherEOIConfirmation)
                : wxPanel(parentWindow)
                , multiVizMgr(multiVizMgr)
                , publisherEOIConfirmation(publisherEOIConfirmation)
                , uuid(uuid)
                , castWorkingMemoryPointer(castWorkingMemoryPointer)
                {
                    //this->SetBackgroundColour(wxColour(0, 255, 0)); // Makes the background green so that it's easy to see

                    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

                    wxButton* b;

                    b = new wxButton(this, ID_BTN_CONFIRM, wxT("Confirm"), wxDefaultPosition, wxSize(125, NIFTiConstants::MIN_BUTTON_SIZE), 0);
                    b->SetBackgroundColour(wxColour(0, 255, 0)); // Green

                    sizer->Add(b, 0, 0, 0);

                    b = new wxButton(this, ID_BTN_REJECT, wxT("Reject"), wxDefaultPosition, wxSize(125, NIFTiConstants::MIN_BUTTON_SIZE), 0);
                    b->SetBackgroundColour(wxColour(255, 0, 0)); // Red
                    sizer->Add(b, 0, 0, 0);

                    this->SetSizer(sizer);

                    // Connects the buttons to their onPressed methods
                    Connect(ID_BTN_CONFIRM, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ElementOfInterestPanel::onBtnConfirmPressed));
                    Connect(ID_BTN_REJECT, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ElementOfInterestPanel::onBtnRejectPressed));
                }

                void ElementOfInterestPanel::onBtnConfirmPressed(wxCommandEvent& evt)
                {
                    //printf("onBtnConfirmPressed\n");

                    publishConfirmationMessage(uuid, castWorkingMemoryPointer, eu_nifti_env::ElementOfInterest::STATUS_CONFIRMED);

                    // When the message will get re-published, change color or delete appropriately
                    // For now, let's not do this yet because it would complicate things

                    multiVizMgr->hideElementOfInterestPanel();
                }

                void ElementOfInterestPanel::onBtnRejectPressed(wxCommandEvent& evt)
                {
                    //printf("onBtnRejectPressed\n");

                    publishConfirmationMessage(uuid, castWorkingMemoryPointer, eu_nifti_env::ElementOfInterest::STATUS_REJECTED);

                    // When the message will get re-published, change color or delete appropriately
                    // For now, let's not do this yet because it would complicate things

                    multiVizMgr->hideElementOfInterestPanel();
                }

                void ElementOfInterestPanel::publishConfirmationMessage(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, int8_t status)
                {
                    eu_nifti_env_msg_ros::ElementOfInterestConfirmationMessage confirmationMsg;

                    confirmationMsg.header.stamp = ros::Time::now();
                    confirmationMsg.uuid = uuid;
                    confirmationMsg.status = status;
                    confirmationMsg.castWorkingMemoryPointer = castWorkingMemoryPointer;

                    publisherEOIConfirmation.publish(confirmationMsg);

                    //ROS_INFO("Published a ElementOfInterestConfirmationMessage for uuid #%i with status %i", uuid, status);
                }
                
            }
        }
    }
}