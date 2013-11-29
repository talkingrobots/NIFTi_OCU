// Benoit 2012-05-31
// rostopic pub -1 /flippers_cmd nifti_robot_driver_msgs/FlippersState -- -1.75 -1.75 1.75 1.75

#include <wx/bitmap.h>
#include <wx/bmpbuttn.h>
#include <wx/gbsizer.h>
#include <wx/slider.h>
#include <wx/statbmp.h>
//#include <wx/stattext.h>

#include <ros/node_handle.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <nifti_ocu_msgs/RobotControl.h>

#include <nifti_robot_driver_msgs/FlippersState.h>

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"
#include "RobotsStatusesManager.h"

#include "UGVActuatorsControlPanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const short ID_FLIPPER_FRONT_LEFT = 3;
                const short ID_FLIPPER_FRONT_RIGHT = 4;
                const short ID_FLIPPER_REAR_LEFT = 5;
                const short ID_FLIPPER_REAR_RIGHT = 6;

                const char* TOPIC_FLIPPER_CONTROL = "/flippers_cmd";
                const char* TOPIC_USER_ACTIONS = "ocu/robotControl";
                const char* RESET_FLIPPERS_TOPIC = "/enable";
                const char* DIFFERENTIAL_BRAKE_TOPIC = "/brake";
                const char* ROBOT_STATUS_TOPIC = "/robot_status";

                const std::string IMG_OPEN_FILE_NAME = "DifferentialUnlocked.png";
                const std::string IMG_CLOSED_FILE_NAME = "DifferentialLocked.png";
                const std::string IMG_LASER_ROTATION_FILE_NAME = "LaserRotation.png";

                const double FLIPPER_OPEN = 0;
                const double FLIPPER_CLOSED_BOTTOM = 3.5;
                const double FLIPPER_CLOSED_TOP = 3.15; // Warning: this prevents the robot from moving because the flippers rub against one another
                const double FLIPPER_VERTICAL = 1.75;
                const double FLIPPER_APPROACH = 0.5;
                const double FLIPPER_AMORTIZATION = -0.5;

                UGVActuatorsControlPanel::UGVActuatorsControlPanel(wxWindow *parentWindow, ros::Publisher* publisherUserActions)
                : wxPanel(parentWindow)
                , publisherUserActions(publisherUserActions)
                {
                    //this->SetBackgroundColour(wxColour(0, 255, 0)); // Makes the background green so that it's easy to see
                    
                    publisherFlipperPositions = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<nifti_robot_driver_msgs::FlippersState > (TOPIC_FLIPPER_CONTROL, 4);
                    publisherFlipperReset = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Bool > (RESET_FLIPPERS_TOPIC, 1);
                    publisherDifferentialBrakePosition = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Bool > (DIFFERENTIAL_BRAKE_TOPIC, 1);

                    wxGridBagSizer* gridBagSizer = new wxGridBagSizer();

                    wxBitmap imgDriving, imgBanana, imgFlat, imgClimbStart, imgClimbEnd;
                    loadImages(imgDriving, imgBanana, imgFlat, imgClimbStart, imgClimbEnd);

                    // Adds some glue to the left             
                    //                    gridBagSizer->Add(
                    //                            new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(8, -1), 0),
                    //                            wxGBPosition(0, 0));

                    gridBagSizer->Add(
                            new wxBitmapButton(this, nifti_ocu_msgs::RobotControl::POSITION_DRIVING, imgDriving, wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0),
                            wxGBPosition(0, 1));

                    gridBagSizer->Add(
                            new wxBitmapButton(this, nifti_ocu_msgs::RobotControl::POSITION_BANANA, imgBanana, wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0),
                            wxGBPosition(0, 2));

                    gridBagSizer->Add(
                            new wxBitmapButton(this, nifti_ocu_msgs::RobotControl::POSITION_FLAT, imgFlat, wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0),
                            wxGBPosition(0, 3));

                    gridBagSizer->Add(
                            new wxBitmapButton(this, nifti_ocu_msgs::RobotControl::POSITION_CLIMB_START, imgClimbStart, wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0),
                            wxGBPosition(0, 4));

                    gridBagSizer->Add(
                            new wxBitmapButton(this, nifti_ocu_msgs::RobotControl::POSITION_CLIMB_END, imgClimbEnd, wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0),
                            wxGBPosition(0, 5));


                    sliderBrake = new wxSlider(this, nifti_ocu_msgs::RobotControl::SLIDER_BRAKE, 0, 0, 1, wxDefaultPosition, wxSize(96, NIFTiConstants::MIN_BUTTON_SIZE), wxSL_HORIZONTAL);
                    btnEnableFlippers = new wxButton(this, nifti_ocu_msgs::RobotControl::BTN_RESET_FLIPPERS, wxT("!"), wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0);
                    btnEnableFlippers->Enable(false); // By default, assumes that the flippers are ok

                    std::string completeImagePathOpen = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + IMG_OPEN_FILE_NAME;
                    std::string completeImagePathClosed = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + IMG_CLOSED_FILE_NAME;

                    wxStaticBitmap *bmpOpen = new wxStaticBitmap(this, nifti_ocu_msgs::RobotControl::OPEN, wxBitmap(wxString(completeImagePathOpen.c_str(), wxConvUTF8), wxBITMAP_TYPE_PNG), wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0);
                    wxStaticBitmap *bmpClosed = new wxStaticBitmap(this, nifti_ocu_msgs::RobotControl::CLOSE, wxBitmap(wxString(completeImagePathClosed.c_str(), wxConvUTF8), wxBITMAP_TYPE_PNG), wxDefaultPosition, wxSize(NIFTiConstants::MIN_BUTTON_SIZE, NIFTiConstants::MIN_BUTTON_SIZE), 0);

                    //                    // Adds some glue to the left
                    //                    gridBagSizer->Add(
                    //                            new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(8, -1), 0),
                    //                            wxGBPosition(1, 0));

                    gridBagSizer->Add(
                            bmpOpen,
                            wxGBPosition(1, 1));

                    gridBagSizer->Add(
                            sliderBrake,
                            wxGBPosition(1, 2), wxGBSpan(1, 2));

                    gridBagSizer->Add(
                            bmpClosed,
                            wxGBPosition(1, 4));

                    gridBagSizer->Add(
                            btnEnableFlippers,
                            wxGBPosition(1, 5));


                    
                    int ID_BTN_LASER_ROTATION=12345;
                    
                    bool showDebugTools = false;
                    if (NIFTiROSUtil::getParam("showDebugTools", showDebugTools) && showDebugTools)
                    {
                        btnLaserRotation = new wxButton(this, ID_BTN_LASER_ROTATION, wxT("Start Rotation"), wxDefaultPosition, wxSize(-1, NIFTiConstants::MIN_BUTTON_SIZE), 0);

                        gridBagSizer->Add(
                                btnLaserRotation,
                                wxGBPosition(2, 1),
                                wxGBSpan(1, 5));
                        
                        Connect(ID_BTN_LASER_ROTATION, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnLaserRotationPressed));
                        
                        publisherLaserRotationStart = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Float64 > ("/scanning_speed_cmd", 1);
                        publisherLaserRotationStop = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Bool > ("/laser_center", 1);
                    }


                    this->SetSizer(gridBagSizer);

                    // Connects the buttons to the callback methods

                    Connect(nifti_ocu_msgs::RobotControl::POSITION_DRIVING, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnDrivingPressed));
                    Connect(nifti_ocu_msgs::RobotControl::POSITION_BANANA, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnBananaPressed));
                    Connect(nifti_ocu_msgs::RobotControl::POSITION_FLAT, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnFlatPressed));
                    Connect(nifti_ocu_msgs::RobotControl::POSITION_CLIMB_START, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnClimbStartPressed));
                    Connect(nifti_ocu_msgs::RobotControl::POSITION_CLIMB_END, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnClimbEndPressed));

                    bmpOpen->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(UGVActuatorsControlPanel::onIconClick), NULL, this);
                    bmpClosed->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(UGVActuatorsControlPanel::onIconClick), NULL, this);
                    btnEnableFlippers->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(UGVActuatorsControlPanel::onBtnResetFlippersPressed), NULL, this);
                    sliderBrake->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(UGVActuatorsControlPanel::onScrollDifferentialBrake), NULL, this);

                    RobotsStatusesManager::addListener(this);
                }

                UGVActuatorsControlPanel::~UGVActuatorsControlPanel()
                {
                    RobotsStatusesManager::removeListener(this);
                }

                void UGVActuatorsControlPanel::onRobotStatusUpdated(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg)
                {
                    //printf("onStatusMsgReceived\n");


                    if (msg.get()->brake_on != nifti_ocu_msgs::RobotControl::OPEN && msg.get()->brake_on != nifti_ocu_msgs::RobotControl::CLOSE)
                    {
                        ROS_WARN("Received an incorrect status for the robot brake: %i", msg.get()->brake_on);
                        return;
                    }

                    // 1) Moves the slider to represent the status of the brake
                    sliderBrake->SetValue(msg.get()->brake_on);


                    // 2) Checks if a flipper is disabled to turn on or off the reset button

                    // From the ELMO controller documentation:
                    // SR - Status Register:  Bit #4 is Motor ON
                    // I do a bitwise AND to read bit #4
                    const bool ALL_FLIPPERS_ON =
                            msg.get()->controllers_status.flipper_front_left & 16 &&
                            msg.get()->controllers_status.flipper_front_right & 16 &&
                            msg.get()->controllers_status.flipper_rear_left & 16 &&
                            msg.get()->controllers_status.flipper_rear_right & 16;

                    btnEnableFlippers->Enable(!ALL_FLIPPERS_ON);
                    if (!ALL_FLIPPERS_ON)
                    {
                        btnEnableFlippers->SetBackgroundColour(wxColour(255, 0, 0)); // Red
                    }
                    else
                    {
                        btnEnableFlippers->SetBackgroundColour(this->GetBackgroundColour());
                    }

                }

                void UGVActuatorsControlPanel::onBtnDrivingPressed(wxCommandEvent& evt)
                {
                    publishFlipperPositions(-FLIPPER_CLOSED_TOP, -FLIPPER_CLOSED_TOP, FLIPPER_VERTICAL, FLIPPER_VERTICAL);
                    publishUserAction(evt.GetId());
                }

                void UGVActuatorsControlPanel::onBtnBananaPressed(wxCommandEvent& evt)
                {
                    publishFlipperPositions(-FLIPPER_APPROACH, -FLIPPER_APPROACH, FLIPPER_APPROACH, FLIPPER_APPROACH);
                    publishUserAction(evt.GetId());
                }

                void UGVActuatorsControlPanel::onBtnFlatPressed(wxCommandEvent& evt)
                {
                    publishFlipperPositions(-FLIPPER_OPEN, -FLIPPER_OPEN, FLIPPER_OPEN, FLIPPER_OPEN);
                    publishUserAction(evt.GetId());
                }

                void UGVActuatorsControlPanel::onBtnClimbStartPressed(wxCommandEvent& evt)
                {
                    publishFlipperPositions(FLIPPER_OPEN, FLIPPER_OPEN, FLIPPER_AMORTIZATION, FLIPPER_AMORTIZATION);
                    publishUserAction(evt.GetId());
                }

                void UGVActuatorsControlPanel::onBtnClimbEndPressed(wxCommandEvent& evt)
                {
                    publishFlipperPositions(-FLIPPER_AMORTIZATION, -FLIPPER_AMORTIZATION, FLIPPER_OPEN, FLIPPER_OPEN);
                    publishUserAction(evt.GetId());
                }

                void UGVActuatorsControlPanel::onBtnResetFlippersPressed(wxCommandEvent& evt)
                {
                    publishFlipperReset();
                    publishUserAction(nifti_ocu_msgs::RobotControl::BTN_RESET_FLIPPERS);
                }

                void UGVActuatorsControlPanel::onScrollDifferentialBrake(wxScrollEvent& evt)
                {
                    publishDifferentialBrakePosition(sliderBrake->GetValue());
                    publishUserAction(nifti_ocu_msgs::RobotControl::SLIDER_BRAKE + sliderBrake->GetValue());
                }

                void UGVActuatorsControlPanel::onIconClick(wxMouseEvent& event)
                {
                    sliderBrake->SetValue(event.GetId());
                    publishDifferentialBrakePosition(event.GetId());
                    publishUserAction(nifti_ocu_msgs::RobotControl::SLIDER_BRAKE + event.GetId());
                }
                
                void UGVActuatorsControlPanel::onBtnLaserRotationPressed(wxCommandEvent& evt)
                {
                    if(btnLaserRotation->GetLabel() == wxT("Start Rotation"))
                    {
                        std_msgs::Float64 msg;
                        msg.data = 1.2; // Benoit: Seems like a reasonable value from the Y2 End-user evaluation bags
                        btnLaserRotation->SetLabel(wxT("Stop Rotation"));
                        publisherLaserRotationStart.publish(msg);
                    }
                    else
                    {
                        std_msgs::Bool msg;
                        msg.data = true;
                        btnLaserRotation->SetLabel(wxT("Start Rotation"));
                        publisherLaserRotationStop.publish(msg);
                    }
                }

                ////////////////////////////////
                ////////////////////////////////
                // ACTUAL WORK METHODS BELOW

                void UGVActuatorsControlPanel::publishFlipperPositions(double frontLeft, double frontRight, double rearLeft, double rearRight)
                {
                    nifti_robot_driver_msgs::FlippersState msg;
                    msg.frontLeft = frontLeft;
                    msg.frontRight = frontRight;
                    msg.rearLeft = rearLeft;
                    msg.rearRight = rearRight;
                    publisherFlipperPositions.publish(msg);
                }

                void UGVActuatorsControlPanel::publishFlipperReset()
                {
                    std_msgs::Bool msg;
                    msg.data = true;
                    publisherFlipperReset.publish(msg);
                }

                void UGVActuatorsControlPanel::publishDifferentialBrakePosition(bool locked)
                {
                    std_msgs::Bool msg;
                    msg.data = locked;
                    publisherDifferentialBrakePosition.publish(msg);
                }

                void UGVActuatorsControlPanel::publishUserAction(int actionID)
                {
                    // Publishes a message to tell that the user activated a tool
                    nifti_ocu_msgs::RobotControl msgOCU;
                    msgOCU.stamp = ros::Time::now();
                    msgOCU.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msgOCU.widgetUsed = actionID;
                    publisherUserActions->publish(msgOCU);
                }

                void UGVActuatorsControlPanel::loadImages(wxBitmap& imgDriving, wxBitmap& imgBanana, wxBitmap& imgFlat, wxBitmap& imgClimbStart, wxBitmap& imgClimbEnd)
                {
                    bool success;
                    std::string completeImagePath;

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "PositionDriving.png";
                    success = imgDriving.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "PositionBanana.png";
                    success = imgBanana.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "PositionFlat.png";
                    success = imgFlat.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "PositionClimbStart.png";
                    success = imgClimbStart.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "PositionClimbEnd.png";
                    success = imgClimbEnd.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);
                }

            }
        }
    }
}