// Benoit 2011-03-04

#include <sstream>

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/statbmp.h>
#include <wx/string.h>
#include <wx/timer.h>
#include <wx/toolbar.h>

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"
#include "RobotsStatusesManager.h"
#include "IMultiVisualizationManager.h"

#include "Panels/SimplifiedDialoguePanel.h"

#include "Panels/TopPanel.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const int TopPanel::NO_INFO_TIMEOUT = 2; // Waits a few seconds before displaying that no signal has been received

                const char* TopPanel::TOPIC_BATTERY = "/monitoring/batteryInfo";
                const char* TopPanel::TOPIC_WIFI = "/monitoring/wifiLink";

                const char* TopPanel::BATTERY_LABEL = "Battery: ";
                const char* TopPanel::WIFI_LABEL = "WiFi: ";

                const std::string ICON_BATTERY_NO_INFO = "Battery_no_info.png";
                const std::string ICON_SIGNAL_NO_INFO = "Signal_no_info.png";

                TopPanel::TopPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr)
                : wxPanel(parentWindow)
                , multiVizMgr(multiVizMgr)
                , levelBattery(-1)
                , levelSignal(-1)
                , timeLastMessageBattery(0)
                , timeLastMessageSignal(0)
                {
                    //this->SetBackgroundColour(wxColour(255, 0, 0)); // Benoit Todo Remove this TEMP

                    wxBoxSizer* boxSizer = new wxBoxSizer(wxHORIZONTAL);

                    toolBarForVizModes = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxSize(225, 60), wxNO_BORDER | wxTB_HORIZONTAL);
                    boxSizer->Add(toolBarForVizModes, 0, wxALL, 0);

                    initToolBarForVizModes();

                    wxStaticText* glueLeft = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(10, -1), 0);
                    boxSizer->Add(glueLeft, 0, wxALL, 0);


                    // Adds the Dialogue Panel, or expandable glue
                    bool showDialoguePanel = false;
                    if (NIFTiROSUtil::getParam("showDialoguePanel", showDialoguePanel) && showDialoguePanel)
                    {
                        const int HEIGHT = 60;
                        SimplifiedDialoguePanel* dialoguePanel = new SimplifiedDialoguePanel(this);
                        dialoguePanel->SetMinSize(wxSize(-1, HEIGHT));
                        boxSizer->Add(dialoguePanel, 1, wxALIGN_CENTER_VERTICAL | wxALL, 0);
                    }
                    else
                    {
                        wxStaticText* glueDialog = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(-1, -1), 0);
                        boxSizer->Add(glueDialog, 1, wxALL, 0);
                    }


                    wxStaticText* glueRight = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(10, -1), 0);
                    boxSizer->Add(glueRight, 0, wxALL, 0);

                    ////////////////////////////////
                    //
                    // Initializes the icons for battery & WiFi
                    //
                    ////////////////////////////////

                    loadImagesBatteryAndSignal();

                    bmpBattery = new wxStaticBitmap(this, -1, bmpBatteryNoInfo);
                    bmpBattery->SetToolTip(wxString("No Info", wxConvUTF8));
                    boxSizer->Add(bmpBattery, 0, wxALIGN_BOTTOM | wxBOTTOM, 5);

                    bmpSignal = new wxStaticBitmap(this, -1, bmpSignalNoInfo);
                    bmpSignal->SetToolTip(wxString("No Info", wxConvUTF8));
                    boxSizer->Add(bmpSignal, 0, wxALL, 10);

                    lblTimeOfDay = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(132, 48), 0); // Todo: improve these hard-coded values
                    updateLabelTime();
                    boxSizer->Add(lblTimeOfDay, 0, wxALIGN_BOTTOM, 0);

                    // Makes the time of day bigger
                    wxFont font = lblTimeOfDay->GetFont();
                    font.SetPointSize(32);
                    lblTimeOfDay->SetFont(font);

                    ////////////////

                    this->SetSizer(boxSizer);
                    this->Layout();


                    ////////////////////////////////
                    //
                    // Subscribes to ROS for battery and WiFi info
                    //
                    ////////////////////////////////


                    // Subscribes to the battery and wifi topics and links the call back methods
                    subscriberBattery = nh.subscribe(TOPIC_BATTERY, 1, &TopPanel::onBatteryMsgReceived, this);
                    RobotsStatusesManager::addListener(this);
                    subscriberWiFi = nh.subscribe(TOPIC_WIFI, 1, &TopPanel::onWiFiMsgReceived, this);

                    // Timer to update the display every second
                    timerUpdate = new wxTimer(this);
                    Connect(timerUpdate->GetId(), wxEVT_TIMER, wxTimerEventHandler(TopPanel::onTimer));
                    timerUpdate->Start(1000, false);
                }

                void TopPanel::loadImagesBatteryAndSignal()
                {
                    std::string completeImagePath;
                    bool success;

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_no_info" + ".png";
                    success = bmpBatteryNoInfo.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_0" + ".png";
                    success = bmpBattery0.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_1" + ".png";
                    success = bmpBattery1.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_2" + ".png";
                    success = bmpBattery2.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_3" + ".png";
                    success = bmpBattery3.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Battery_4" + ".png";
                    success = bmpBattery4.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_no_info" + ".png";
                    success = bmpSignalNoInfo.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_0" + ".png";
                    success = bmpSignal0.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_1" + ".png";
                    success = bmpSignal1.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_2" + ".png";
                    success = bmpSignal2.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_3" + ".png";
                    success = bmpSignal3.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "Signal_4" + ".png";
                    success = bmpSignal4.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);
                }

                void loadImagesVizModes(wxBitmap& imgSingle, wxBitmap& imgDualHorizontal, wxBitmap& imgDualVertical, wxBitmap& imgQuad)
                {
                    bool success;
                    std::string completeImagePath;

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "VizModeSingle.png";
                    success = imgSingle.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "VizModeDualHorizontal.png";
                    success = imgDualHorizontal.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "VizModeDualVertical.png";
                    success = imgDualVertical.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + "VizModeQuad.png";
                    success = imgQuad.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);
                }

                void TopPanel::initToolBarForVizModes()
                {
                    wxBitmap imgSingle, imgDualHorizontal, imgDualVertical, imgQuad;
                    loadImagesVizModes(imgSingle, imgDualHorizontal, imgDualVertical, imgQuad);

                    toolBarForVizModes->AddRadioTool(toolBarForVizModes->GetToolsCount(), wxString(), imgSingle, wxNullBitmap, wxString(wxT("Single Display")));
                    toolBarForVizModes->AddRadioTool(toolBarForVizModes->GetToolsCount(), wxString(), imgDualHorizontal, wxNullBitmap, wxString(wxT("Dual Display - Horizontal")));
                    toolBarForVizModes->AddRadioTool(toolBarForVizModes->GetToolsCount(), wxString(), imgDualVertical, wxNullBitmap, wxString(wxT("Dual Display - Vertical")));
                    toolBarForVizModes->AddRadioTool(toolBarForVizModes->GetToolsCount(), wxString(), imgQuad, wxNullBitmap, wxString(wxT("Quad Display")));

                    // This is from when this code was in VisualizationFrame
                    //wxAuiPaneInfo& pane = auiManager->GetPane(toolBarForVizModes);
                    //pane.MinSize(toolBarForVizModes->GetSize());

                    // Todo Check if it will be disconnected by wxWdigets
                    toolBarForVizModes->Connect(wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler(TopPanel::onVizModeSelected), NULL, this);

                    toolBarForVizModes->ToggleTool(3, true); // Makes active the button for Quad Display
                }

                void TopPanel::onVizModeSelected(wxCommandEvent& evt)
                {
                    multiVizMgr->setVizMode(MultiVizMode(evt.GetId()));
                }

                void TopPanel::onBatteryMsgReceived(const monitoring_msgs::BatteryConstPtr& msg)
                {
                    onBatteryLevelReceived( (int) (msg.get()->currentBatteryLevel) );
                    
//                    std::stringstream fullString;
//                    fullString << BATTERY_LABEL << (int) (msg.get()->currentBatteryLevel * 100) << "%";
//                    wxString mystring(fullString.str().c_str(), wxConvUTF8);
//
//                    this->lblBattery->SetLabel(mystring);
//
//                    // Restarts the timer to give another few second before we decide that the connection is lost
//                    timerBattery->Start(NO_INFO_TIMEOUT * 1000, true);
                }

                void TopPanel::onTimer(wxTimerEvent& event)
                {
                    updateIconBattery();
                    updateIconSignal();
                    updateLabelTime();
                }

                void TopPanel::onRobotStatusUpdated(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg)
                {
                    onBatteryLevelReceived( (int) (msg.get()->battery_level) );
                }
                
                void TopPanel::onBatteryLevelReceived(int level)
                {
                    levelBattery = level;
                    timeLastMessageBattery = time(&timeLastMessageBattery);

                    std::stringstream fullString;
                    fullString << BATTERY_LABEL << levelBattery << "%";
                    wxString mystring(fullString.str().c_str(), wxConvUTF8);

                    bmpBattery->SetToolTip(mystring);

                    //                    // If the battery level increased just a little bit, do not change the icon (to avoid flickering)
                    //                    // There would be the bug after a signal loss and the new value is like the old one                    
                    //                    double diff = msg.get()->battery_level - oldValue;
                    //                    printf("Diff: %f\n", diff);
                    //                    
                    //                    if(diff >= 0 && diff < 5) 
                    //                    {
                    //                        printf("Not changing icon\n");
                    //                        return;
                    //                    }
                    //                    
                    //                    oldValue = msg.get()->battery_level;    
                }

                void TopPanel::onWiFiMsgReceived(const monitoring_msgs::WiFiConstPtr& msg)
                {
                    levelSignal = (int) (msg.get()->linkQuality * 100);
                    timeLastMessageSignal = time(&timeLastMessageSignal);

                    updateIconSignal();
                    
//                    std::stringstream fullString;
//                    fullString << WIFI_LABEL << levelSignal << "%";
//                    wxString mystring(fullString.str().c_str(), wxConvUTF8);
//                    
//                    bmpSignal->SetToolTip(mystring);
                }

                void TopPanel::updateIconBattery()
                {
                    //std::cout << "void TopPanel::onBatteryTimer(wxTimerEvent& event) " << levelBattery << " " << timeLastMessageBattery << std::endl;

                    time_t now = time(&now);

                    if (now - timeLastMessageBattery >= NO_INFO_TIMEOUT)
                    {
                        levelBattery = -1;
                        bmpBattery->SetToolTip(wxString("Battery: No Info", wxConvUTF8));
                        bmpBattery->SetBitmap(bmpBatteryNoInfo);
                    }
                    else
                    {
                        if (levelBattery >= 75)
                        {
                            bmpBattery->SetBitmap(bmpBattery4);
                        }
                        else if (levelBattery >= 50)
                        {
                            bmpBattery->SetBitmap(bmpBattery3);
                        }
                        else if (levelBattery >= 25)
                        {
                            bmpBattery->SetBitmap(bmpBattery2);
                        }
                        else if (levelBattery >= 5)
                        {
                            bmpBattery->SetBitmap(bmpBattery1);
                        }
                        else
                        {
                            bmpBattery->SetBitmap(bmpBattery0);
                        }
                    }
                }

                void TopPanel::updateIconSignal()
                {
                    time_t now = time(&now);

                    if (now - timeLastMessageSignal >= NO_INFO_TIMEOUT)
                    {
                        levelSignal = -1;
                        bmpSignal->SetToolTip(wxString("Signal: No Info", wxConvUTF8));
                        bmpSignal->SetBitmap(bmpSignalNoInfo);
                    }
                    else
                    {
                        if (levelSignal >= 75)
                        {
                            bmpSignal->SetBitmap(bmpSignal4);
                        }
                        else if (levelSignal >= 50)
                        {
                            bmpSignal->SetBitmap(bmpSignal3);
                        }
                        else if (levelSignal >= 25)
                        {
                            bmpSignal->SetBitmap(bmpSignal2);
                        }
                        else if (levelSignal >= 5)
                        {
                            bmpSignal->SetBitmap(bmpSignal1);
                        }
                        else
                        {
                            bmpSignal->SetBitmap(bmpSignal0);
                        }
                    }
                }

                void TopPanel::updateLabelTime()
                {
                    //time_t rawtime = time(&rawtime); // Real time
                    time_t rawtime = ros::Time::now().sec; // Real time or simulated time (from a ROS bag)
                    char formattedTime[6];
                    strftime(formattedTime, 6, "%H:%M", localtime(&rawtime));

                    this->lblTimeOfDay->SetLabel(wxString(formattedTime, wxConvUTF8));
                }

                TopPanel::~TopPanel()
                {
                    timerUpdate->Stop();

                    delete timerUpdate;

                    RobotsStatusesManager::removeListener(this);
                }

            }
        }
    }


}
