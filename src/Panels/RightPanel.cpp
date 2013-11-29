// Benoit 2011-08-30

#include <wx/button.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

#include <std_msgs/Bool.h>

#include <nifti_ocu_msgs/RobotControl.h>

#include "Panels/ElementOfInterestPanel.h"
#include "Panels/MotorControlPanel.h"
#include "Panels/PictureInfoPanel.h"
#include "Panels/PlanningPanel.h"
#include "Panels/PullHandle.h"
#include "Panels/UGVActuatorsControlPanel.h"

#include "IMultiVisualizationManager.h"

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "Panels/RightPanel.h"
#include "Panels/PictureInfoPanel.h"

#include <eu_nifti_env_msg_ros/ElementOfInterestConfirmationMessage.h>

using namespace std;
using namespace eu::nifti::ocu::gui;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const std::string FILE_NAME_HANDLE = "PullHandle_Flippers.png";

                const std::string ROBOT_CONTROL_BUTTONS_TOPIC = "ocu/robotControl";
                const std::string TOPIC_CONTROL_AUTONOMOUS_FLIPPERS = "autonomous_flippers/control";

                const size_t indexActuatorPanel = 2;

                const int wxUnstretchable = 0;
                const int wxNoBorder = 0;


                RightPanel::RightPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr)
                : wxPanel(parentWindow)
                , multiVizMgr(multiVizMgr)
                , eoiPanel(NULL)
                , pictureInfoPanel(NULL)
                {
                    //this->SetBackgroundColour(wxColour(255, 0, 0)); // Makes the background green so that it's easy to see

                    publisherUserActions = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<nifti_ocu_msgs::RobotControl > (ROBOT_CONTROL_BUTTONS_TOPIC, 1);
                    publisherControlAutonomousFlippers = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Bool > (TOPIC_CONTROL_AUTONOMOUS_FLIPPERS, 1);

                    // The publisher is created here because creating and destroying publishers (directly in the EOIPanel) is not well supported by ROS
                    publisherEOIConfirmation = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<eu_nifti_env_msg_ros::ElementOfInterestConfirmationMessage > ("/eoi_confirmation", 1);

                    sizer = new wxBoxSizer(wxVERTICAL);

                    sizer->Add(new MotorControlPanel(this, &publisherUserActions), wxUnstretchable, wxNoBorder, 0);

                    // Gives a bit of space below the driving dial
                    sizer->AddSpacer(15);

                    sizer->Add(new UGVActuatorsControlPanel(this, &publisherUserActions), wxUnstretchable, wxNoBorder, 0);

                    // Creates the pull handle
                    std::string imagePathHandle = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + FILE_NAME_HANDLE;
                    PullHandle* handle = new PullHandle(this, this, imagePathHandle, imagePathHandle, PullHandle::DOWN, true);
                    handle->setCloseability(true);

                    // Adds the pull handle
                    wxBoxSizer *subsizer = new wxBoxSizer(wxHORIZONTAL);
                    subsizer->AddSpacer(10); // I don't know why, but that makes it perfectly centered
                    subsizer->AddSpacer(1.5 * NIFTiConstants::MIN_BUTTON_SIZE); // I don't know why, but that makes it perfectly centered
                    subsizer->Add(handle, 0, 0, 0);

                    sizer->Add(subsizer, 0, 0, 0);
 
                    sizer->Add(new PlanningPanel(this), wxUnstretchable, wxALIGN_CENTER | wxEXPAND | wxLEFT | wxRIGHT, 5);

                    // Adds some glue
                    sizer->AddStretchSpacer(1);

                    this->SetSizer(sizer);
                    sizer->SetSizeHints(this); // set size hints to honour minimum size
                    this->Layout();
                }

                void RightPanel::onPullHandleClosed(PullHandle* handle)
                {
                    handle->setOpeneability(true);
                    handle->setCloseability(false);

                    sizer->Hide(indexActuatorPanel);
                    sizer->Layout();

                    std_msgs::Bool msg;
                    msg.data = true;
                    publisherControlAutonomousFlippers.publish(msg);

                    publishUserAction(nifti_ocu_msgs::RobotControl::PANEL_FLIPPER_AUTONOMY + nifti_ocu_msgs::RobotControl::CLOSE);
                }

                void RightPanel::onPullHandleOpened(PullHandle* handle)
                {
                    handle->setOpeneability(false);
                    handle->setCloseability(true);

                    sizer->Show(indexActuatorPanel);
                    sizer->Layout();

                    std_msgs::Bool msg;
                    msg.data = false;
                    publisherControlAutonomousFlippers.publish(msg);

                    publishUserAction(nifti_ocu_msgs::RobotControl::PANEL_FLIPPER_AUTONOMY + nifti_ocu_msgs::RobotControl::OPEN);
                }

                void RightPanel::onPullHandleClicked(PullHandle* handle)
                {

                }

                void RightPanel::showElementOfInterestPanel(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer)
                {
                    assert(eoiPanel == NULL);

                    eoiPanel = new ElementOfInterestPanel(this, multiVizMgr, uuid, castWorkingMemoryPointer, publisherEOIConfirmation);
                    sizer->Insert(4, eoiPanel, wxUnstretchable, wxNoBorder, 0); // Inserts at the fourth position, just above the plan

                    this->Layout();
                }

                void RightPanel::hideElementOfInterestPanel()
                {
                    // If the panel is already hidden, then do nothing
                    if (eoiPanel == NULL) return;

                    sizer->Detach(eoiPanel);
                    this->RemoveChild(eoiPanel);
                    delete eoiPanel;
                    eoiPanel = NULL;

                    this->Layout();
                }

                void RightPanel::showAnnotatedPictureInfo(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //cout << "void RightPanel::showAnnotatedPictureInfo" << endl;
                    
                    if(picture == NULL)
                    {
                        // If the panel is already hidden, then do nothing
                        if (pictureInfoPanel != NULL)
                        {
                            sizer->Detach(pictureInfoPanel);
                            this->RemoveChild(pictureInfoPanel);
                            delete pictureInfoPanel;
                            pictureInfoPanel = NULL;

                            this->Layout();
                        }
                    }
                    else
                    {                   
                        // If the panel didn't already exist, create it
                        if (pictureInfoPanel == NULL)
                        {
                            pictureInfoPanel = new PictureInfoPanel(this);
                            sizer->Add(pictureInfoPanel, wxUnstretchable, wxALIGN_CENTER | wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 5);
                            this->Layout();
                        }
                        
                        pictureInfoPanel->showAnnotatedPictureInfo(picture);
                    }
                    
                    //this->SetMinSize(wxSize(-1, parentWindow->GetSize().GetHeight()));
                    
                    // Just to test
                    //this->Update();
                    //this->Layout();
                    //this->Refresh();
                }
                
                void RightPanel::publishUserAction(int actionID)
                {
                    // Publishes a message to tell that the user activated a tool
                    nifti_ocu_msgs::RobotControl msgOCU;
                    msgOCU.stamp = ros::Time::now();
                    msgOCU.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msgOCU.widgetUsed = actionID;
                    publisherUserActions.publish(msgOCU);
                }

            }
        }
    }
}