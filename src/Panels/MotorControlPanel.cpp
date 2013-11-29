// Benoit 2010-06-11
// rostopic pub -1 pioneer/cmd_vel geometry_msgs/Twist '[10, 0, 0]' '[0, 0, 0]'

#include <ros/package.h>

#include <math.h>

#include <wx/dcclient.h> // for wxPaintDC
#include <wx/statbmp.h>
#include <wx/bitmap.h>

#include <nifti_ocu_msgs/RobotControl.h>

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "MotorControlPanel.h"


namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                // Todo Benoit: Makes the red lines move with the finger (so that they intersect at the click)

                const std::string MotorControlPanel::IMAGE_FILE_NAME = "DrivingWidget.png";

                const int MotorControlPanel::CONTROL_WIDGET_RADIUS = 128;

                const double MotorControlPanel::MAX_LINEAR_SPEED = 0.6; // 0.6 m/s  // This seems to be the maximum speed at which the robot goes
                const double MotorControlPanel::MAX_ANGULAR_SPEED = 0.6; // 0.6 rad/s // This is just to copy the value above, and it's quite fast for turning

                const int MotorControlPanel::ROS_PUBLISHING_RATE = 10;
                const char* MotorControlPanel::ROS_PUBLISHING_TOPIC = "/cmd_vel";

                const int MotorControlPanel::EMERGENCY_BRAKE = 666;

                /**
                 * I think that the way in which I handle the threads is fine, but it could
                 * leak, and it could be multi-thread unsafe
                 * @param parent
                 */
                MotorControlPanel::MotorControlPanel(wxWindow* parent, ros::Publisher* robotControlButtonsPublisher)
                : wxPanel(parent, wxID_ANY)
                , robotControlButtonsPublisher(robotControlButtonsPublisher)
                , keepSending(false)
                {
                    // Creates the object that will publish on the appropriate topic (buffer of length 1)
                    publisherCmdVel = rosNodeHandle.advertise<geometry_msgs::Twist > (ROS_PUBLISHING_TOPIC, 1);

//                    Connect(wxEVT_PAINT, wxPaintEventHandler(MotorControlPanel::onPaint), 0, this);
                    this->Connect(wxEVT_LEAVE_WINDOW, wxMouseEventHandler(MotorControlPanel::onMouseLeave), NULL, this);
                    
                    std::string completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + IMAGE_FILE_NAME;
                    wxBitmap background;

                    // Loads the background image
                    bool success = background.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    wxStaticBitmap* sb = new wxStaticBitmap(this, -1, background);

                    sb->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MotorControlPanel::onMouseDown), NULL, this);
                    sb->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(MotorControlPanel::onMouseUp), NULL, this);
                    sb->Connect(wxEVT_MOTION, wxMouseEventHandler(MotorControlPanel::onMouseMove), NULL, this);
                    sb->Connect(wxEVT_LEFT_DCLICK, wxMouseEventHandler(MotorControlPanel::onMouseDoubleClick), NULL, this);
                    
                    
                    // Initializes the object to be published to ROS to speed 0
                    desiredMotion.linear.x = 0;
                    desiredMotion.linear.y = 0;
                    desiredMotion.linear.z = 0;

                    desiredMotion.angular.x = 0;
                    desiredMotion.angular.y = 0;
                    desiredMotion.angular.z = 0;

                    this->SetMaxSize(wxSize(2 * CONTROL_WIDGET_RADIUS, 2 * CONTROL_WIDGET_RADIUS));
                    this->SetMinSize(wxSize(2 * CONTROL_WIDGET_RADIUS, 2 * CONTROL_WIDGET_RADIUS));
                }

                void MotorControlPanel::onMouseDown(wxMouseEvent& event)
                {
                    //calculateDesiredMotionForCircle(event);
                    calculateDesiredMotionForTwoStageCircle(event);
                    this->startSending();
                }

                void MotorControlPanel::onMouseUp(wxMouseEvent& event)
                {
                    this->stopSending();
                }

                void MotorControlPanel::onMouseMove(wxMouseEvent& event)
                {
                    // If the mouse button is not pressed, then do nothing
                    if (!event.m_leftDown)
                        return;

                    //calculateDesiredMotionForCircle(event);
                    calculateDesiredMotionForTwoStageCircle(event);
                }

                /**
                 * This should not be called while a publishing thread to make the robot move is running
                 * @param event
                 */
                void MotorControlPanel::onMouseDoubleClick(wxMouseEvent& event)
                {
                    desiredMotion.linear.z = EMERGENCY_BRAKE;
                    publisherCmdVel.publish(desiredMotion);
                    desiredMotion.linear.z = 0;

                    //ROS_INFO("I published [e-brake]");
                }
                
                void MotorControlPanel::onMouseLeave(wxMouseEvent& event)
                {
                    this->stopSending();
                }
                
                void MotorControlPanel::calculateDesiredMotionForTwoStageCircle(wxMouseEvent& event)
                {
                    int distanceFromCenterX = event.m_x - CONTROL_WIDGET_RADIUS;
                    int distanceFromCenterY = CONTROL_WIDGET_RADIUS - event.m_y;

                    // Converts the mouse coordinates to polar coordinates
                    double angle = atan((double) distanceFromCenterY / (double) distanceFromCenterX);


                    // The amplitude is how far the click was from the center
                    double percentageRadius = sqrt(distanceFromCenterX * distanceFromCenterX + distanceFromCenterY * distanceFromCenterY) / sqrt(CONTROL_WIDGET_RADIUS * CONTROL_WIDGET_RADIUS);

                    // Max-out the percentage at 1 (if the user's finger goes out of the widget)
                    //percentageRadius = percentageRadius > 1 ? 1 : percentageRadius;
                    if(percentageRadius > 1)
                    {
                        this->stopSending(); // This will publish a 0,0 command
                        return;
                    }


                    double amplitude;

                    // Calculates the amplitude with a rate of 0.5 up to 60 %, then steeper
                    if (percentageRadius <= 0.6) // 60 % is the ratio of the inner circle in the image
                    {
                        amplitude = (0.4 / 0.6) * percentageRadius;
                    }
                    else
                    {
                        amplitude = 0.4 + (0.6 / 0.4)*(percentageRadius - 0.6);
                    }


                    desiredMotion.linear.x = amplitude * MAX_LINEAR_SPEED * sin(angle); // TODO CHANGE MAXIMA
                    desiredMotion.angular.z = amplitude * -MAX_ANGULAR_SPEED * cos(angle);

                    // Reverses the angle if the click was in the bottom part (to feel like a car steering wheel)
                    // Now, it does not reverse anymore, because people found that not natural
                    if (distanceFromCenterY < 0)
                    {
                        //desiredMotion.angular.z *= -1;
                    }

                    // Reverses the angle if the click was in the left part (because of arc tan)
                    if (distanceFromCenterX < 0)
                    {
                        desiredMotion.linear.x *= -1;
                        desiredMotion.angular.z *= -1;
                    }


//                    printf("****************\n");
//                    printf("CONTROL_WIDGET_RADIUS: %i\n", CONTROL_WIDGET_RADIUS);
//                    printf("event.m_x: %i\n", event.m_x);
//                    printf("event.m_y: %i\n", event.m_y);
//                    printf("distanceFromCenterX: %i\n", distanceFromCenterX);
//                    printf("distanceFromCenterY: %i\n", distanceFromCenterY);
//                    printf("angle: %f\n", angle);
//                    printf("percentageRadius: %f\n", percentageRadius);
//                    printf("amplitude: %f\n", amplitude);
//                    printf("desiredMotion.linear.x: %f\n", desiredMotion.linear.x);
//                    printf("desiredMotion.angular.z: %f\n", desiredMotion.angular.z);
//                    printf("****************\n");


                }

                void MotorControlPanel::startSending()
                {
                    keepSending = true;

                    publishingThread = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&MotorControlPanel::publishToROS, this)));
                }

                void MotorControlPanel::publishToROS()
                {
                    nifti_ocu_msgs::RobotControl msgUserAction;

                    
                    // Publishes a message to tell that the user started using the widget
                    msgUserAction.stamp = ros::Time::now();
                    msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msgUserAction.widgetUsed = nifti_ocu_msgs::RobotControl::WIDGET_MOTION + nifti_ocu_msgs::RobotControl::START;
                    robotControlButtonsPublisher->publish(msgUserAction);
                    
                    ros::Rate loop_rate(ROS_PUBLISHING_RATE);

                    while (keepSending == true)
                    {

                        publisherCmdVel.publish(desiredMotion);

                        //ROS_INFO("I published this motor command: (%f, %f)", desiredMotion.linear.x, desiredMotion.angular.z);

                        loop_rate.sleep();

                    }

                    // Sends a speed of 0 to stop the robot
                    desiredMotion.linear.x = 0;
                    desiredMotion.angular.z = 0;
                    publisherCmdVel.publish(desiredMotion);

                    
                    // Publishes a message to tell that the user stopped using the widget
                    msgUserAction.stamp = ros::Time::now();
                    msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msgUserAction.widgetUsed = nifti_ocu_msgs::RobotControl::WIDGET_MOTION + nifti_ocu_msgs::RobotControl::STOP;
                    robotControlButtonsPublisher->publish(msgUserAction);
                    
                }

                void MotorControlPanel::stopSending()
                {
                    if(keepSending == false)
                    {
                        return;
                    }
                    
                    keepSending = false;

                    // This could freeze the UI if the publisher is stuck or something
                    publishingThread.get()->join();
                }


            }
        }
    }
}