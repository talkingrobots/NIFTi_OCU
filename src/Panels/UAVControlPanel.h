// Benoit 2010-06-11

#ifndef NIFTI_UAV_CONTROL_PANEL_H
#define NIFTI_UAV_CONTROL_PANEL_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <wx/panel.h>
#include <wx/bitmap.h>

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

            class MotorCommandPublisher;

            namespace gui
            {

                /**
                 * Publishes commands for the motor in a separate thread. WARNING, it is not
                 * thread safe, so only start/stop should be called in sequence.
                 */
                class UAVControlPanel : public wxPanel
                {
                public:

                    UAVControlPanel(wxWindow* parent);
                    ~UAVControlPanel();

                protected:
                    void onPaint(wxPaintEvent& evt);
                    wxBitmap background;

                private:

                    ros::NodeHandle rosNodeHandle;
                    ros::Publisher publisher;

                    void startSending();
                    void stopSending();

                    void onMouseDown(wxMouseEvent& event);
                    void onMouseUp(wxMouseEvent& event);
                    void onMouseMove(wxMouseEvent& event);

                    void publishToROS();

                    // Calculates the desired linear and angular speeds based on the cursor position
                    void calculateDesiredMotionForScales(wxMouseEvent& event);

                    // Flag for the publishing thread
                    volatile bool keepSending;

                    // Object that will be published to the ROS core
                    geometry_msgs::Twist desiredMotion;

                    // Thread for the worker that will publish at a given frequency
                    //boost::thread* publishingThread;
                    boost::shared_ptr<boost::thread> publishingThread;

                    // File path for the control image
                    static const std::string IMAGE_PATH;

                    // Size of the control in pixels
                    static const int CONTROL_WIDGET_SIZE;

                    // Number of pixels from which the user can click and the click will still be considered to be on the axis
                    static const int ALLOWED_DISTANCE_FROM_AXIS;

                    // Maximum speeds at which the UAV can strafe left and right and go up and down
                    static const double MAX_HORIZONTAL_SPEED;
                    static const double MAX_VERTICAL_SPEED;

                    // For the robot P3-AT, publishing at 10 Hz is sufficient
                    static const int ROS_PUBLISHING_RATE;
                    static const char* ROS_PUBLISHING_TOPIC;


                };

            }
        }
    }
}



#endif
