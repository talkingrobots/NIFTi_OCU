// Benoit 2012-05-31

#ifndef EU_NIFTI_OCU_GUI_UGV_ACTUATORS_CONTROL_PANEL_H_
#define EU_NIFTI_OCU_GUI_UGV_ACTUATORS_CONTROL_PANEL_H_

#include <wx/panel.h>

#include <ros/publisher.h>

#include "IRobotStatusListener.h"

class wxBitmap;
class wxSlider;

namespace ros
{
    class Publisher;
}

namespace eu
{

    namespace nifti
    {

        namespace ocu
        {

            namespace gui
            {

                /**
                 * Displays widgets to control the robot's actuators: flippers and differential brakes
                 */
                class UGVActuatorsControlPanel : public wxPanel, public IRobotStatusListener
                {
                public:
                    UGVActuatorsControlPanel(wxWindow *parentWindow, ros::Publisher* publisherUserActions);
                    ~UGVActuatorsControlPanel();

                protected:
                    void onBtnResetFlippersPressed(wxCommandEvent& evt);
                    void onScrollDifferentialBrake(wxScrollEvent& evt);
                    void onIconClick(wxMouseEvent& event);

                    void onRobotStatusUpdated(const nifti_robot_driver_msgs::RobotStatusStampedConstPtr& msg);

                    void onBtnBananaPressed(wxCommandEvent& evt);
                    void onBtnFlatPressed(wxCommandEvent& evt);
                    void onBtnClimbStartPressed(wxCommandEvent& evt);
                    void onBtnDrivingPressed(wxCommandEvent& evt);
                    void onBtnClimbEndPressed(wxCommandEvent& evt);
                    
                    void onBtnLaserRotationPressed(wxCommandEvent& evt);

                    void publishFlipperPositions(double frontLeft, double frontRight, double rearLeft, double rearRight);
                    void publishFlipperReset();
                    void publishDifferentialBrakePosition(bool locked);
                    void publishUserAction(int actionID);
    
                    wxSlider* sliderBrake;
                    wxButton* btnEnableFlippers;
                    
                    // Only for debugging
                    wxButton *btnLaserRotation;
                    ros::Publisher publisherLaserRotationStart;
                    ros::Publisher publisherLaserRotationStop;

                    ros::Publisher publisherFlipperPositions;
                    ros::Publisher publisherFlipperReset;
                    ros::Publisher publisherDifferentialBrakePosition;
                    ros::Publisher *publisherUserActions;

                private:
                    void loadImages(wxBitmap& imgDriving, wxBitmap& imgBanana, wxBitmap& imgFlat, wxBitmap& imgClimbStart, wxBitmap& imgClimbEnd);

                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_GUI_UGV_ACTUATORS_CONTROL_PANEL_H_ */