// Benoit 2011-10-13

#include <wx/event.h>

//#include <OGRE/OgreMath.h>
#include <OGRE/OgreMatrix3.h>

#include "IVisualizationManager.h"

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "ViewControllers/VirtualPTZViewController.h"
#include "NIFTiConstants.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                const double VirtualPTZViewController::PAN_OFFSET = 54.0; // The pan-offset due to the Point-Grey ladybug

                const double VirtualPTZViewController::MAX_UP = 45.0; // It can get confusing if more is allowed
                const double VirtualPTZViewController::MAX_DOWN = -45.0; // It can get confusing if more is allowed

                const double VirtualPTZViewController::MAX_LEFT = 1.0/0.0; // Infinity to allow multiple revolutions.
                const double VirtualPTZViewController::MAX_RIGHT = -1.0/0.0; // Infinity to allow multiple revolutions.
                //                const double VirtualPTZViewController::MAX_LEFT = +90.0 + PAN_OFFSET;
                //                const double VirtualPTZViewController::MAX_RIGHT = -90.0 + PAN_OFFSET;

                const double VirtualPTZViewController::MAX_FOV = 120.0; // The algorithm can't go much beyond that, because the image gets distorted
                const double VirtualPTZViewController::MIN_FOV = 22.5; // The image gets highly pixelated beyond that, and it's the same value as the first-person view

                // These two lines are from FirstPersonViewController
                //const float FirstPersonViewController::FIELD_OF_VIEW_MAX = Ogre::Math::HALF_PI; // pi/2 radians, 90  degrees (it's too difficult to see beyond that)
                //const float FirstPersonViewController::FIELD_OF_VIEW_MIN = Ogre::Math::PI / 8; // pi/8 radians, approx. 22.5 degrees (it's zoomed in quite far already)

                const double VirtualPTZViewController::DEFAULT_PAN = 0 + PAN_OFFSET;
                const double VirtualPTZViewController::DEFAULT_TILT = -5.625; // Same as First-Person view, because it seems more natural
                const double VirtualPTZViewController::DEFAULT_FOV = 90.0; // This seems to be the value for the human eye
                const int VirtualPTZViewController::VIEWPORT_WIDTH = 512; // This value works, but it's not calibrated

                const double VirtualPTZViewController::PAN_SPEED = 0.0015; // To be more finely calibrated with the touch screen
                const double VirtualPTZViewController::TILT_SPEED = 0.001; // To be more finely calibrated with the touch screen
                const double VirtualPTZViewController::ZOOM_SPEED = -.001; // To be more finely calibrated with the touch screen

                const char* VirtualPTZViewController::ROS_PUBLISHING_TOPIC = "/viz/ptz/config";

                VirtualPTZViewController::VirtualPTZViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : CameraViewController(sceneMgr, vizMgr, camera, frameTransformer, "/map")
                {
                    // Todo: move all this to activate()

                    // Creates the object that will publish on the appropriate topic (buffer of length 1)
                    publisher = rosNodeHandle.advertise<omnicamera_msgs::VirtualCameraConfig > (ROS_PUBLISHING_TOPIC, 1, true);

                    // Todo Adjust these values
                    virtualPTZmsg.viewportWidth = VIEWPORT_WIDTH;
                    virtualPTZmsg.viewportHeight = VIEWPORT_WIDTH;
                    virtualPTZmsg.imageUpdateInterval = 1.0; // This value works, but it's not calibrated
                    // These values never change, and the others are set in activate()

                }
                
                VirtualPTZViewController::~VirtualPTZViewController()
                {
                }

                void VirtualPTZViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    if (evt.evt.LeftDown() || evt.evt.LeftDClick()) // wxWidgets sometimes sends DClick events instead of LeftDown
                    {
                        const wxPoint lastPosition = evt.evt.GetPosition();
                        lastX = lastPosition.x;
                        lastY = lastPosition.y;
                    }
                    else if (evt.evt.Dragging())
                    {
                        const wxPoint currentPosition = evt.evt.GetPosition();
                        updateOrientation(currentPosition.x, currentPosition.y);

                        publisher.publish(virtualPTZmsg);
                        vizMgr->updateOCUInfoViewContent();
                        //ROS_INFO("I published [%f, %f] %i\n", virtualPTZmsg.pan, virtualPTZmsg.tilt, publisher.getNumSubscribers());

                        // Updates the old position
                        lastX = currentPosition.x;
                        lastY = currentPosition.y;
                    } 
                    else if (evt.evt.GetWheelRotation() != 0) // Handles the zoom with the mouse wheel
                    {
                        int diff = evt.evt.GetWheelRotation();
                        double fov = virtualPTZmsg.horizontalFov + diff * ZOOM_SPEED * virtualPTZmsg.horizontalFov;

                        if (fov > MAX_FOV) fov = MAX_FOV;
                        if (fov < MIN_FOV) fov = MIN_FOV;

                        virtualPTZmsg.horizontalFov = fov;
                        virtualPTZmsg.verticalFov = fov;

                        publisher.publish(virtualPTZmsg);
                        vizMgr->updateOCUInfoViewContent();
                        //ROS_INFO("New FOV: %f", virtualPTZmsg.horizontalFov);
                    }
                }

                std::string VirtualPTZViewController::toString() const
                {
                    // todo Implement std::string VirtualPTZViewController::toString()
                    return std::string();
                }
                
                double VirtualPTZViewController::getFieldOfViewHorizontal() const
                {
                    return virtualPTZmsg.horizontalFov * NIFTiConstants::PI / 180;
                }
                
                double VirtualPTZViewController::getFieldOfViewVertical() const
                {
                    return virtualPTZmsg.verticalFov * NIFTiConstants::PI / 180;
                }

                void VirtualPTZViewController::lookAt(const Ogre::Vector3& point)
                {
                    // todo Implement void VirtualPTZViewController::lookAt(const Ogre::Vector3& point)
                }

                void VirtualPTZViewController::resetView()
                {
                    virtualPTZmsg.pan = DEFAULT_PAN;
                    virtualPTZmsg.tilt = DEFAULT_TILT;
                    virtualPTZmsg.horizontalFov = DEFAULT_FOV;
                    virtualPTZmsg.verticalFov = DEFAULT_FOV;

                    publisher.publish(virtualPTZmsg);
                    vizMgr->updateOCUInfoViewContent();

                    //ROS_INFO("I published [PTZ resetView] %i\n", publisher.getNumSubscribers());
                }

                ////////////////////////////////
                ////////////////////////////////

                /**
                 * This behaviour is not ideal if the user goes out of range while dragging,
                 * and comes back. todo To be improved.
                 * @param currentX
                 * @param currentY
                 */
                void VirtualPTZViewController::updateOrientation(int currentX, int currentY)
                {
                    // Calculate distance from last position
                    int motionX = currentX - lastX;
                    int motionY = currentY - lastY;

                    // Move the camera accordingly
                    virtualPTZmsg.pan += motionX * PAN_SPEED * virtualPTZmsg.horizontalFov;
                    virtualPTZmsg.tilt += motionY * TILT_SPEED * virtualPTZmsg.horizontalFov;

                    // Ensures that the limits are not surpassed (left right are at infinity right now)
                    //if (virtualPTZmsg.pan >= MAX_LEFT)
                    //    virtualPTZmsg.pan = MAX_LEFT;
                    //else if (virtualPTZmsg.pan <= MAX_RIGHT)
                    //    virtualPTZmsg.pan = MAX_RIGHT;
                    if (virtualPTZmsg.tilt >= MAX_UP)
                        virtualPTZmsg.tilt = MAX_UP;
                    else if (virtualPTZmsg.tilt <= MAX_DOWN)
                        virtualPTZmsg.tilt = MAX_DOWN;
                }

            }
        }
    }
}
