// Benoit August 2010

#include <wx/event.h>

#include "IVisualizationManager.h"
#include "NIFTiConstants.h"

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "ViewControllers/ArmCamViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                const double ArmCamViewController::MAX_UP = 1105;
                const double ArmCamViewController::MAX_DOWN = 200;
                const double ArmCamViewController::MAX_LEFT = -NIFTiConstants::PI_OVER_2;
                const double ArmCamViewController::MAX_RIGHT = NIFTiConstants::PI_OVER_2;
                const double ArmCamViewController::MAX_TILT_UP = -NIFTiConstants::PI/3;
                const double ArmCamViewController::MAX_TILT_DOWN = NIFTiConstants::PI_OVER_2;
                const double ArmCamViewController::HOME_POSITION_HEIGHT = 200;
                const double ArmCamViewController::HOME_POSITION_PAN = 0;
                const double ArmCamViewController::HOME_POSITION_TILT = 0;


                const double ArmCamViewController::PIXEL_TO_MOVEMENT_RATIO_HEIGHT = 2.25;
                const double ArmCamViewController::PIXEL_TO_MOVEMENT_RATIO_PAN = 0.0025;
                const double ArmCamViewController::PIXEL_TO_MOVEMENT_RATIO_TILT = 0.0002;

                const char* ArmCamViewController::ROS_TOPIC_HEIGHT = "/arm/height";
                const char* ArmCamViewController::ROS_TOPIC_PAN = "/pan_controller/command";
                const char* ArmCamViewController::ROS_TOPIC_TILT = "/tilt_controller/command";

                ArmCamViewController::ArmCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : CameraViewController(sceneMgr, vizMgr, camera, frameTransformer, "/openni_rgb_optical_frame")
                {
                    // Todo: move all this to activate

                    // Creates the object that will publish on the appropriate topic (buffer of length 1)
                    publisherHeight = rosNodeHandle.advertise<nifti_arm_msgs::msg_height > (ROS_TOPIC_HEIGHT, 1);
                    publisherPan = rosNodeHandle.advertise<std_msgs::Float64 > (ROS_TOPIC_PAN, 1);
                    publisherTilt = rosNodeHandle.advertise<std_msgs::Float64 > (ROS_TOPIC_TILT, 1);

                    resetView();
                }

                ArmCamViewController::~ArmCamViewController()
                {
                }

                void ArmCamViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {

                    if (evt.evt.LeftDown())
                    {
                        const wxPoint lastPosition = evt.evt.GetPosition();
                        lastX = lastPosition.x;
                        lastY = lastPosition.y;
                    }
                    else if (evt.evt.Dragging())
                    {
                        const wxPoint currentPosition = evt.evt.GetPosition();
                        publishDueToMouseMotion(currentPosition.x, currentPosition.y);
                    }
                    else if (evt.evt.GetWheelRotation() != 0) // Handles the zoom with the mouse wheel
                    {
                        publishDueToMouseWheel(evt.evt.GetWheelRotation());
                    }
                }

                double ArmCamViewController::getFieldOfViewHorizontal() const
                {
                    // This is for the Asus Xtion 
                    return 1.01229097; // 58 degrees, found on the web
                }
                
                double ArmCamViewController::getFieldOfViewVertical() const
                {
                    // This is for the Asus Xtion 
                    return 0.785398163; // 45 degrees, found on the web
                }

                void ArmCamViewController::lookAt(const Ogre::Vector3& point)
                {
                    // todo Implement void PTZCamViewController::lookAt(const Ogre::Vector3& point)
                }

                void ArmCamViewController::resetView()
                {
                    // Initializes the object to be published to ROS to speed 0
                    desiredHeight.height = HOME_POSITION_HEIGHT;
                    desiredPan.data = HOME_POSITION_PAN;
                    desiredTilt.data = HOME_POSITION_TILT;

                    publisherHeight.publish(desiredHeight);
                    publisherPan.publish(desiredPan);
                    publisherTilt.publish(desiredTilt);

                    //ROS_INFO("I published [PTU HOME]\n");
                }

                std::string ArmCamViewController::toString() const
                {
                    // todo Implement std::string PTZCamViewController::toString()
                    return std::string();
                }

                ////////////////////////////////
                ////////////////////////////////

                /**
                 * This behaviour is not ideal if the user goes out of range while dragging,
                 * and comes back. todo To be improved.
                 * @param currentX
                 * @param currentY
                 */
                void ArmCamViewController::publishDueToMouseMotion(int currentX, int currentY)
                {
                    // Calculate distance from last position
                    int motionX = currentX - lastX;
                    int motionY = currentY - lastY;

                    // Move the camera accordingly
                    desiredHeight.height += PIXEL_TO_MOVEMENT_RATIO_HEIGHT * motionY;
                    desiredPan.data += PIXEL_TO_MOVEMENT_RATIO_PAN * motionX;


                    // Ensures that the limits are not surpassed
                    if (desiredPan.data <= MAX_LEFT)
                        desiredPan.data = MAX_LEFT;
                    else if (desiredPan.data >= MAX_RIGHT)
                        desiredPan.data = MAX_RIGHT;
                    if (desiredHeight.height >= MAX_UP)
                        desiredHeight.height = MAX_UP;
                    else if (desiredHeight.height <= MAX_DOWN)
                        desiredHeight.height = MAX_DOWN;

                    //printf("New Position X: %f, New Position Y: %f\n", desiredPosition.position[PAN_SLOT], desiredPosition.position[TILT_SLOT]);

                    // Update the "old position"
                    lastX = currentX;
                    lastY = currentY;

                    publisherHeight.publish(desiredHeight);
                    publisherPan.publish(desiredPan);

                    //ROS_INFO("I published [%f, %f]\n", desiredPosition.position[PAN_SLOT], desiredPosition.position[TILT_SLOT]);
                }

                void ArmCamViewController::publishDueToMouseWheel(int scrollAmount)
                {
                    desiredTilt.data += scrollAmount * PIXEL_TO_MOVEMENT_RATIO_TILT;

                    if (desiredTilt.data > MAX_TILT_DOWN) desiredTilt.data = MAX_TILT_DOWN;
                    if (desiredTilt.data < MAX_TILT_UP) desiredTilt.data = MAX_TILT_UP;

                    publisherTilt.publish(desiredTilt);
                }

                float ArmCamViewController::getFieldOfView()
                {
                    // Todo Fill out this value when we know what camera will be on the arm
                    return -1;
                }

            }
        }
    }
}
