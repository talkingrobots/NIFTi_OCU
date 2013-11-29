// Benoit 2010-06-11

#include <math.h>

#include <boost/filesystem.hpp>     // These two lines for fs::path
namespace fs = boost::filesystem; // These two lines for fs::path

#include <wx/dcclient.h> // for wxPaintDC

#include <ros/node_handle.h>

#include "NIFTiConstants.h"

#include "UAVControlPanel.h"


namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const double PI = 3.141592;

                const std::string UAVControlPanel::IMAGE_PATH = "media/images/CartesianScales.png";

                const int UAVControlPanel::CONTROL_WIDGET_SIZE = 128;

                const int UAVControlPanel::ALLOWED_DISTANCE_FROM_AXIS = 16;

                const double UAVControlPanel::MAX_HORIZONTAL_SPEED = 1;
                const double UAVControlPanel::MAX_VERTICAL_SPEED = 1;

                const int UAVControlPanel::ROS_PUBLISHING_RATE = 10;
                const char* UAVControlPanel::ROS_PUBLISHING_TOPIC = "/uav/cmd_vel"; //This is a made-up topic

                /**
                 * I think that the way in which I handle the threads is fine, but it could
                 * leak, and it could be multi-thread unsafe
                 * @param parent
                 */
                UAVControlPanel::UAVControlPanel(wxWindow* parent)
                : wxPanel(parent, wxID_ANY)
                , keepSending(false)
                {
                    // Creates the object that will publish on the appropriate topic (buffer of length 1)
                    publisher = rosNodeHandle.advertise<geometry_msgs::Twist > (ROS_PUBLISHING_TOPIC, 1);

                    Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(UAVControlPanel::onMouseDown), NULL, this);
                    Connect(wxEVT_LEFT_UP, wxMouseEventHandler(UAVControlPanel::onMouseUp), NULL, this);
                    Connect(wxEVT_MOTION, wxMouseEventHandler(UAVControlPanel::onMouseMove), NULL, this);

                    Connect(wxEVT_PAINT, wxPaintEventHandler(UAVControlPanel::onPaint), 0, this);


                    std::string completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/CartesianScales.png";

                    // Loads the background image
                    background.LoadFile(wxString::FromAscii(completeImagePath.c_str()));

                    // Initializes the object to be published to ROS to speed 0
                    desiredMotion.linear.x = 0;
                    desiredMotion.linear.y = 0;
                    desiredMotion.linear.z = 0;

                    desiredMotion.angular.x = 0;
                    desiredMotion.angular.y = 0;
                    desiredMotion.angular.z = 0;
                }

                UAVControlPanel::~UAVControlPanel()
                {
                }

                void UAVControlPanel::onMouseDown(wxMouseEvent& event)
                {
                    calculateDesiredMotionForScales(event);
                    this->startSending();
                }

                void UAVControlPanel::onMouseUp(wxMouseEvent& event)
                {
                    this->stopSending();
                }

                void UAVControlPanel::onMouseMove(wxMouseEvent& event)
                {
                    // If the mouse button is not pressed, then do nothing
                    if (!event.m_leftDown)
                        return;

                    calculateDesiredMotionForScales(event);
                }

                /**
                 * Draws the image (the scales) in the background
                 * @param evt
                 */
                void UAVControlPanel::onPaint(wxPaintEvent& evt)
                {
                    wxPaintDC dc(this);
                    dc.DrawBitmap(background, 0, 0);
                }

                /**
                 * Calculates the motion that the user desires based on the position of
                 * the cursor and the size of the square frame.
                 * @param event
                 */
                void UAVControlPanel::calculateDesiredMotionForScales(wxMouseEvent& event)
                {
                    // Gets the size of the control panel
                    const wxSize panelSize = this->GetSize();

                    // Gets the position of the mouse w.r.t. the axes
                    int horizontalPositionOfClick = (event.m_x - CONTROL_WIDGET_SIZE);
                    int verticalPositionOfClick = (CONTROL_WIDGET_SIZE - event.m_y);

                    // Applies limits to the four sides (cannot go faster than max speed)
                    if (horizontalPositionOfClick > CONTROL_WIDGET_SIZE)
                        horizontalPositionOfClick = CONTROL_WIDGET_SIZE;
                    if (horizontalPositionOfClick < -CONTROL_WIDGET_SIZE)
                        horizontalPositionOfClick = -CONTROL_WIDGET_SIZE;
                    if (verticalPositionOfClick > CONTROL_WIDGET_SIZE)
                        verticalPositionOfClick = CONTROL_WIDGET_SIZE;
                    if (verticalPositionOfClick < -CONTROL_WIDGET_SIZE)
                        verticalPositionOfClick = -CONTROL_WIDGET_SIZE;

                    // Gets the distance of the click from the axes (to force the user to click along the axis)
                    int distanceFromVerticalAxis = horizontalPositionOfClick < 0 ? -horizontalPositionOfClick : horizontalPositionOfClick;
                    int distanceFromHorizontalAxis = verticalPositionOfClick < 0 ? -verticalPositionOfClick : verticalPositionOfClick;

                    // Ensures that the click was close to the scale
                    if (distanceFromVerticalAxis > ALLOWED_DISTANCE_FROM_AXIS && distanceFromHorizontalAxis > ALLOWED_DISTANCE_FROM_AXIS)
                    {
                        //printf("OUT of the limits %i %i\n", distanceFromVerticalAxis, distanceFromHorizontalAxis);
                        return;
                    }

                    // Ensure that the click was not directly in the center
                    if (distanceFromVerticalAxis <= ALLOWED_DISTANCE_FROM_AXIS && distanceFromHorizontalAxis <= ALLOWED_DISTANCE_FROM_AXIS)
                    {
                        //printf("IN the limits %i %i\n", distanceFromVerticalAxis, distanceFromHorizontalAxis);
                        return;
                    }

                    // If the click is closer to the vertical axis than to the horizontal axis
                    if (distanceFromVerticalAxis > distanceFromHorizontalAxis)
                    {
                        // Updates the object to be published with the desired motion
                        desiredMotion.linear.x = MAX_HORIZONTAL_SPEED * horizontalPositionOfClick / CONTROL_WIDGET_SIZE;
                        desiredMotion.linear.z = 0;
                        //printf("HOR %i %i %f %f\n", distanceFromVerticalAxis, distanceFromHorizontalAxis, desiredMotion.linear.x, desiredMotion.linear.z);
                    }
                    else
                    {
                        // Updates the object to be published with the desired motion
                        desiredMotion.linear.x = 0;
                        desiredMotion.linear.z = MAX_VERTICAL_SPEED * verticalPositionOfClick / CONTROL_WIDGET_SIZE;
                        //printf("VER %i %i %f %f\n", distanceFromVerticalAxis, distanceFromHorizontalAxis, desiredMotion.linear.x, desiredMotion.linear.z);
                    }

                }

                void UAVControlPanel::startSending()
                {
                    keepSending = true;

                    publishingThread = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&UAVControlPanel::publishToROS, this)));
                }

                void UAVControlPanel::publishToROS()
                {
                    ros::Rate loop_rate(ROS_PUBLISHING_RATE);

                    while (keepSending == true)
                    {
                        publisher.publish(desiredMotion);
                        loop_rate.sleep();
                    }

                    // Sends a speed of 0 to stop the robot
                    desiredMotion.linear.x = 0;
                    desiredMotion.linear.z = 0;
                    publisher.publish(desiredMotion);

                }

                void UAVControlPanel::stopSending()
                {
                    keepSending = false;

                    // This could freeze the UI if the publisher is stuck or something
                    publishingThread.get()->join();
                }


            }
        }
    }
}


//    inline double min ( double a, double b )
//    {
//        return (b>a)?a:b;
//    }
//
//    inline double max ( double a, double b )
//    {
//        return (b<a)?a:b;
//    }
