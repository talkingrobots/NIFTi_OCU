// Benoit 2011-03-04

#ifndef NIFTI_TOP_PANEL_H_
#define NIFTI_TOP_PANEL_H_

#include <wx/panel.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <monitoring_msgs/Battery.h>
#include <monitoring_msgs/WiFi.h>

#include "IRobotStatusListener.h"

class wxToolBar;
class wxStaticText;
class wxTimer;
class wxTimerEvent;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IMultiVisualizationManager;
            
            namespace gui
            {

                /**
                 * Panel that goes at the top of the screen
                 */
                class TopPanel : public wxPanel, public IRobotStatusListener
                {
                public:
                    TopPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr);
                    ~TopPanel();

                protected:
                    void initToolBarForVizModes();
                    
                    void onVizModeSelected(wxCommandEvent& evt);
                    
                    void loadImagesBatteryAndSignal();
                    
                    void onBatteryMsgReceived(const monitoring_msgs::BatteryConstPtr& msg);
                    void onWiFiMsgReceived(const monitoring_msgs::WiFiConstPtr& msg);

                    void onTimer(wxTimerEvent& event);
                    
                    void onRobotStatusUpdated(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg);
                    
                    void updateIconBattery();
                    void updateIconSignal();
                    void updateLabelTime();
                    
                    void onBatteryLevelReceived(int level);

                    IMultiVisualizationManager* multiVizMgr;
                    
                    wxToolBar* toolBarForVizModes;

                    wxStaticText* lblTimeOfDay;
                    
                    wxStaticBitmap* bmpBattery;
                    wxStaticBitmap* bmpSignal;

                    wxTimer* timerUpdate;

                    ros::NodeHandle nh;
                    ros::Subscriber subscriberBattery;
                    ros::Subscriber subscriberWiFi;

                    wxBitmap bmpBatteryNoInfo, bmpBattery0, bmpBattery1, bmpBattery2, bmpBattery3, bmpBattery4;
                    wxBitmap bmpSignalNoInfo, bmpSignal0, bmpSignal1, bmpSignal2, bmpSignal3, bmpSignal4;

                    int levelBattery, levelSignal;
                    time_t timeLastMessageBattery, timeLastMessageSignal;
                    
                    static const int NO_INFO_TIMEOUT;
                    static const char* TOPIC_BATTERY;
                    static const char* TOPIC_WIFI;
                    static const char* BATTERY_LABEL;
                    static const char* WIFI_LABEL;

                };

            }
        }
    }
}

#endif /* NIFTI_TOP_PANEL_H_ */