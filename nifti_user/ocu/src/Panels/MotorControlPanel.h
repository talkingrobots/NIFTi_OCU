// Benoit 2010-06-11

#ifndef EU_NIFTI_OCU_GUI_MOTOR_CONTROL_PANEL_H
#define EU_NIFTI_OCU_GUI_MOTOR_CONTROL_PANEL_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <wx/panel.h>

namespace rviz
{
    class VisualizationManager;
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
                 * Publishes commands for the motor in a separate thread. WARNING, it is not
                 * thread safe, so only start/stop should be called in sequence.
                 */
                class MotorControlPanel : public wxPanel
                {
                public:

                    MotorControlPanel(wxWindow* parent, ros::Publisher* robotControlButtonsPublisher);

                protected:

                    void startSending();
                    void stopSending();

                    void onMouseDown(wxMouseEvent& event);
                    void onMouseUp(wxMouseEvent& event);
                    void onMouseMove(wxMouseEvent& event);
                    void onMouseDoubleClick(wxMouseEvent& event);
                    void onMouseLeave(wxMouseEvent& event);

                    void publishToROS();

                    /**
                     * Calculates the motion that the user desires based on the position of
                     * the cursor within a circular control of a given size.
                     * @param event
                     */
                    void calculateDesiredMotionForTwoStageCircle(wxMouseEvent& event);

                    ros::NodeHandle rosNodeHandle;
                    ros::Publisher publisherCmdVel;
                    ros::Publisher* robotControlButtonsPublisher;

                    // Flag for the publishing thread
                    volatile bool keepSending;

                    // Object that will be published to the ROS core
                    geometry_msgs::Twist desiredMotion;

                    // Thread for the worker that will publish at a given frequency
                    boost::shared_ptr<boost::thread> publishingThread;

                    // File path for the control image
                    static const std::string IMAGE_FILE_NAME;

                    // Size of the control in pixels
                    static const int CONTROL_WIDGET_RADIUS;

                    // For the robot P3-AT, this is a convenient speed (it moves fast enough but not too fast)
                    // In the FLASH memory of the robot, it's also 750 mm/s
                    static const double MAX_LINEAR_SPEED; // 0.75 m/s
                    static const double MAX_ANGULAR_SPEED; // 0.75 rad/s

                    static const int ROS_PUBLISHING_RATE; // The NIFTi robot keeps going at the last received speed
                    static const char* ROS_PUBLISHING_TOPIC;

                    // Special code to be sent to the driver for emergency braking
                    static const int EMERGENCY_BRAKE;

                };

            }
        }
    }
}



#endif
