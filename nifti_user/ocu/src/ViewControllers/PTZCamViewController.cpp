// Benoit August 2010

#include <wx/event.h>

#include "IVisualizationManager.h"

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "ViewControllers/PTZCamViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                const int PTZCamViewController::PAN_SLOT = 0;
                const int PTZCamViewController::TILT_SLOT = 1;

                const double PTZCamViewController::MAX_UP = 0.5;
                const double PTZCamViewController::MAX_DOWN = -0.8;
                const double PTZCamViewController::MAX_LEFT = -2.7;
                const double PTZCamViewController::MAX_RIGHT = 2.7;
                const double PTZCamViewController::HOME_POSITION = 0;

                const double PTZCamViewController::VELOCITY = 0.5;

                const double PTZCamViewController::PIXEL_TO_MOVEMENT_RATIO = 0.0009; // Configuration for the P3-AT at DFKI

                const char* PTZCamViewController::ROS_PUBLISHING_TOPIC = "ptu/cmd";

                PTZCamViewController::PTZCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : CameraViewController(sceneMgr, vizMgr, camera, frameTransformer, "/head_camera")
                {
                    // Todo: move all this to activate
                    
                    // Creates the object that will publish on the appropriate topic (buffer of length 1)
                    publisher = rosNodeHandle.advertise<sensor_msgs::JointState > (ROS_PUBLISHING_TOPIC, 1);

                    // Initializes the two arrays (I don't know how to do it in a better way)
                    desiredPosition.position.resize(2);
                    desiredPosition.velocity.resize(2);

                    // This ensures that the desired position gets reached at this velocity
                    desiredPosition.velocity[PAN_SLOT] = VELOCITY;
                    desiredPosition.velocity[TILT_SLOT] = VELOCITY;
                }
                
                PTZCamViewController::~PTZCamViewController()
                {
                }

                void PTZCamViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    bool moved = false;

                    if (evt.evt.LeftDown())
                    {
                        const wxPoint lastPosition = evt.evt.GetPosition();
                        lastX = lastPosition.x;
                        lastY = lastPosition.y;
                    }
                    else if (evt.evt.Dragging())
                    {
                        moved = true;

                        
                    }

                    if (moved)
                    {
                        const wxPoint currentPosition = evt.evt.GetPosition();
                        publishNewPosition(currentPosition.x, currentPosition.y);
                    }
                }

                std::string PTZCamViewController::toString() const
                {
                    // todo Implement std::string PTZCamViewController::toString()
                    return std::string();
                }

                double PTZCamViewController::getFieldOfViewHorizontal() const
                {
                    return 0.994837674; // 57 degrees for the Kinect. Found on the web
                }
                
                double PTZCamViewController::getFieldOfViewVertical() const
                {
                    return 0.750491578; // 43 degrees for the Kinect. Found on the web
                }
                
                void PTZCamViewController::lookAt(const Ogre::Vector3& point)
                {
                    // todo Implement void PTZCamViewController::lookAt(const Ogre::Vector3& point)
                }
                
                void PTZCamViewController::resetView()
                {
                    // Initializes the object to be published to ROS to speed 0
                    desiredPosition.position[PAN_SLOT] = HOME_POSITION;
                    desiredPosition.position[TILT_SLOT] = HOME_POSITION;

                    publisher.publish(desiredPosition);

                    //ROS_INFO("I published [PTU HOME]\n");
                }

                ////////////////////////////////
                ////////////////////////////////

                /**
                 * This behaviour is not ideal if the user goes out of range while dragging,
                 * and comes back. todo To be improved.
                 * @param currentX
                 * @param currentY
                 */
                void PTZCamViewController::publishNewPosition(int currentX, int currentY)
                {
                    // Calculate distance from last position
                    int motionX = currentX - lastX;
                    int motionY = currentY - lastY;

                    // Move the camera accordingly
                    desiredPosition.position[PAN_SLOT] += PIXEL_TO_MOVEMENT_RATIO * motionX;
                    desiredPosition.position[TILT_SLOT] += PIXEL_TO_MOVEMENT_RATIO * motionY;

                    // Ensures that the limits are not surpassed
                    if (desiredPosition.position[PAN_SLOT] <= MAX_LEFT)
                        desiredPosition.position[PAN_SLOT] = MAX_LEFT;
                    else if (desiredPosition.position[PAN_SLOT] >= MAX_RIGHT)
                        desiredPosition.position[PAN_SLOT] = MAX_RIGHT;
                    if (desiredPosition.position[TILT_SLOT] >= MAX_UP)
                        desiredPosition.position[TILT_SLOT] = MAX_UP;
                    else if (desiredPosition.position[TILT_SLOT] <= MAX_DOWN)
                        desiredPosition.position[TILT_SLOT] = MAX_DOWN;

                    //printf("New Position X: %f, New Position Y: %f\n", desiredPosition.position[PAN_SLOT], desiredPosition.position[TILT_SLOT]);

                    // Update the "old position"
                    lastX = currentX;
                    lastY = currentY;

                    publisher.publish(desiredPosition);

                    //ROS_INFO("I published [%f, %f]\n", desiredPosition.position[PAN_SLOT], desiredPosition.position[TILT_SLOT]);
                }
                
                float PTZCamViewController::getFieldOfView()
                {
                    // This is 55 degress for the Microsoft LifeCam VX-1000
                    return 0.959931089;
                }

            }
        }
    }
}
