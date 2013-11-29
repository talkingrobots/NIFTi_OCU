// Benoit 2011-09-08

#include <stdint.h>
#include <sstream>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "IVisualizationManager.h"
#include "NIFTiConstants.h"

#include "ViewControllers/Chase3DViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                const float Chase3DViewController::VERTICAL_OFFSET = 0.5; //The banana will be at 50 cm from the ground 

                const float Chase3DViewController::RADIUS_BANANA = 3; // I believe that standing 3 meters behind and above the robot is appropriate

                //const float Chase3DViewController::ANGLE_ON_HORIZONTAL_PLANE_INITIAL = 0;
                const float Chase3DViewController::ANGLE_ON_HORIZONTAL_PLANE_INITIAL = Ogre::Math::HALF_PI;
                // Update September 2011: For some reason, the camera would start pointing left at yaw 0
                // This number changed after a bug that appeared in FDDO in January 
                // 2011. For some reason, the map started being displayed turned at 90 deg.
                // Ticket #25
                // Update September 2012: With ROS Fuerte, everything was displayed vertical, so I added pi/2 in several places

                const float Chase3DViewController::ANGLE_ON_BANANA_INITIAL = Ogre::Math::HALF_PI + Ogre::Math::PI / 4;

                const float Chase3DViewController::ANGLE_ON_BANANA_MIN = Ogre::Math::HALF_PI + Ogre::Math::PI / 6; // Zoomed in
                const float Chase3DViewController::ANGLE_ON_BANANA_MAX = Ogre::Math::HALF_PI + Ogre::Math::HALF_PI - Ogre::Math::PI / 8; // Zoomed out

                const float Chase3DViewController::TURN_SPEED = 0.002; // How fast the camera turns around the robot with one pixel mouse movement

                const float Chase3DViewController::ZOOM_SPEED_DRAG = -0.002; // How fast the camera zooms with one pixel mouse movement

                const float Chase3DViewController::ZOOM_SPEED_MOUSE_WHEEL = -0.001; // How fast the camera zooms with one move of the mouse wheel

                Chase3DViewController::Chase3DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : PerspectiveViewController(sceneMgr, vizMgr, camera, frameTransformer, NIFTiConstants::getBaseTFForRobot())
                {
                }

                Chase3DViewController::~Chase3DViewController()
                {
                }

                void Chase3DViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    bool moved = false;

                    if (evt.evt.Dragging())
                    {
                        int32_t diff_x = evt.evt.GetX() - evt.lastX;
                        int32_t diff_y = evt.evt.GetY() - evt.lastY;

                        if (evt.evt.LeftIsDown())
                        {
                            angleOnHorizontalPlane += diff_x * TURN_SPEED;
                            normalizeAngleOnHorizontalPlane();

                            moveAlongBanana(diff_y * ZOOM_SPEED_DRAG);
                        }

                        moved = true;
                    }
                    else if (evt.evt.GetWheelRotation() != 0)
                    {
                        moveAlongBanana(evt.evt.GetWheelRotation() * ZOOM_SPEED_MOUSE_WHEEL);

                        moved = true;
                    }

                    if (moved)
                    {
                        //printf("%s", toString().c_str());
                        updateCamera();

                        evt.vizMgr->queueRender();
                    }

                }

                void Chase3DViewController::lookAt(const Ogre::Vector3& point)
                {
                    // Todo implement properly (need to move the camera perhaps, not just turn it)
                    camera->lookAt(point);
                }

                void Chase3DViewController::resetView()
                {
                    angleBanana = ANGLE_ON_BANANA_INITIAL;
                    angleOnHorizontalPlane = ANGLE_ON_HORIZONTAL_PLANE_INITIAL;

                    updateCamera();
                }

                void Chase3DViewController::moveAlongBanana(float angle)
                {
                    angleBanana += angle;
                    normalizeAngleOnBanana();

                    updateCamera();
                }

                void Chase3DViewController::normalizeAngleOnHorizontalPlane()
                {
                    angleOnHorizontalPlane = fmod(angleOnHorizontalPlane, Ogre::Math::TWO_PI);

                    if (angleOnHorizontalPlane < 0.0f)
                    {
                        angleOnHorizontalPlane = Ogre::Math::TWO_PI + angleOnHorizontalPlane;
                    }
                }

                void Chase3DViewController::normalizeAngleOnBanana()
                {
                    if (angleBanana < ANGLE_ON_BANANA_MIN)
                    {
                        angleBanana = ANGLE_ON_BANANA_MIN;
                    }
                    else if (angleBanana > ANGLE_ON_BANANA_MAX)
                    {
                        angleBanana = ANGLE_ON_BANANA_MAX;
                    }
                }

                void Chase3DViewController::updateCamera()
                {
                    // Calculates the distance away and above the robot based on the position on the banana
                    float distanceBehind = RADIUS_BANANA * cos(angleBanana);
                    float distanceAbove = VERTICAL_OFFSET + (RADIUS_BANANA - (RADIUS_BANANA * sin(angleBanana)));

                    // Calculates the exact position wrt robot as seen from above
                    float dist_x = distanceBehind * sin(angleOnHorizontalPlane);
                    float dist_y = distanceBehind * cos(angleOnHorizontalPlane);

                    // Calculates the pitch based on the position on the banana
                    float pitch = cos(angleBanana) + Ogre::Math::HALF_PI + Ogre::Math::PI / 8;
                    
                    // Calculates the yaw based on the camera position as view from above
                    float yaw = angleOnHorizontalPlane;

                    Ogre::Matrix3 pitchMatrix, yawMatrix;
                    yawMatrix.FromAxisAngle(Ogre::Vector3::UNIT_Z, -Ogre::Radian(yaw));
                    pitchMatrix.FromAxisAngle(Ogre::Vector3::UNIT_X, Ogre::Radian(pitch));

                    // Sets the position and orientation of the camera in Ogre
                    camera->setPosition(Ogre::Vector3(dist_x, dist_y, distanceAbove));
                    camera->setOrientation(yawMatrix * pitchMatrix);

                    vizMgr->updateOCUInfoViewContent();
                }

                std::string Chase3DViewController::toString() const
                {
                    using namespace std;

                    std::ostringstream oss;
                    oss << "Angle on horizontal plane: " << angleOnHorizontalPlane << endl;
                    oss << "Angle on banana: " << angleBanana << endl;
                    oss << "Position: " << camera->getPosition().x << ", " << camera->getPosition().y << ", " << camera->getPosition().z << endl;
                    oss << "Orientation: " << camera->getOrientation().x << ", " << camera->getOrientation().y << ", " << camera->getOrientation().z << ", " << camera->getOrientation().w << endl;

                    return oss.str();
                }


            }
        }
    }
}