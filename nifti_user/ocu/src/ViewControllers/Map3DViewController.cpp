/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Benoit 2011-09-12

#include <stdint.h>
#include <sstream>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "IVisualizationManager.h"

#include "ViewControllers/Map3DViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {
                static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
                        Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
                Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

                const Ogre::Vector3 Map3DViewController::POSITION_INITIAL = Ogre::Vector3(0, 0, 1.5); // At the origin of the map, 1.5 meters above the ground
                const float Map3DViewController::PITCH_INITIAL = Ogre::Math::PI / 16; // Looking a bit below the horizon seems more natural

                const float Map3DViewController::YAW_INITIAL = 0;
                //const float GodViewController::YAW_INITIAL = -Ogre::Math::HALF_PI;
                // Update September 2011: For some reason, the camera would start pointing left at yaw 0
                // This number changed after a bug that appeared in FDDO in January 
                // 2011. For some reason, the map started being displayed turned at 90 deg.
                // Ticket #25
                // Update September 2012: With ROS Fuerte, everything was displayed vertical, so I added pi/2 in several places

                const Ogre::Real HEIGHT_MIN = 0.01; // The camera cannot go below 1cm from the ground
                
                const float Map3DViewController::PITCH_MIN = -Ogre::Math::HALF_PI;
                const float Map3DViewController::PITCH_MAX = Ogre::Math::HALF_PI;

                const float Map3DViewController::PITCH_SPEED = -0.001; // How fast the camera tilts with one pixel mouse movement

                const float Map3DViewController::YAW_SPEED = 0.001; // How fast the camera pans with one pixel mouse movement

                const float Map3DViewController::MOUSE_WHEEL_FACTOR = -0.005; // How fast the camera moves with one move of the mouse wheel

                Map3DViewController::Map3DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : PerspectiveViewController(sceneMgr, vizMgr, camera, frameTransformer, "/map")
                {
                }

                Map3DViewController::~Map3DViewController()
                {
                }

                void Map3DViewController::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
                {
                    //                    printf("Keycode: %i\n", evt.evt.GetKeyCode());
                    //                    printf("RawKeycode: %i\n", evt.evt.GetRawKeyCode());
                    //                    printf("RawKeyflags: %i\n", evt.evt.GetRawKeyFlags());
                    //                    printf("GetId: %i\n", evt.evt.GetId());
                    //                    printf("GetUnicodeKey: %i\n", evt.evt.GetUnicodeKey());
                    //
                    //

                    Ogre::Real currentHeight;
                    switch (evt.evt.GetKeyCode())
                    {
                        case 65: // A
                        case 97: // a
                            //printf("A pressed\n");
                            camera->move(Ogre::Vector3(0,0,0.1));
                            normalizeHeight();
                            break;
                        case 90: // Z
                        case 122: // z
                            //printf("Z pressed\n");
                            camera->move(Ogre::Vector3(0,0,-0.1));
                            normalizeHeight();
                            break;
                        case WXK_LEFT:
                            //printf("WXK_LEFT pressed\n");
                            currentHeight = camera->getPosition().z;
                            camera->moveRelative(Ogre::Vector3(-0.1,0,0));
                            camera->setPosition(camera->getPosition().x, camera->getPosition().y, currentHeight);
                            break;
                        case WXK_RIGHT:
                            //printf("WXK_RIGHT pressed\n");
                            currentHeight = camera->getPosition().z;
                            camera->moveRelative(Ogre::Vector3(0.1,0,0));
                            camera->setPosition(camera->getPosition().x, camera->getPosition().y, currentHeight);
                            break;
                        case WXK_UP:
                            //printf("WXK_UP pressed\n");
                            currentHeight = camera->getPosition().z;
                            camera->moveRelative(Ogre::Vector3(0,0,-0.1));
                            camera->setPosition(camera->getPosition().x, camera->getPosition().y, currentHeight);
                            break;
                        case WXK_DOWN:
                            //printf("WXK_DOWN pressed\n");
                            currentHeight = camera->getPosition().z;
                            camera->moveRelative(Ogre::Vector3(0,0,0.1));
                            camera->setPosition(camera->getPosition().x, camera->getPosition().y, currentHeight);
                            break;
                        case WXK_CONTROL:
                            //printf("WXK_CONTROL pressed. I never receive this event\n");
                            break;
                        case WXK_SHIFT:
                            //printf("WXK_SHIFT pressed. I never receive this event\n");
                            break;
                        default:
                            //printf("%s\n", toString().c_str());
                            evt.evt.Skip();
                            return;
                    }

                }

                void Map3DViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    bool moved = false;

                    if (evt.evt.Dragging())
                    {
                        int32_t diff_x = evt.evt.GetX() - evt.lastX;
                        int32_t diff_y = evt.evt.GetY() - evt.lastY;

                        if (evt.evt.LeftIsDown())
                        {
                            yaw(diff_x * YAW_SPEED);
                            pitch(diff_y * PITCH_SPEED);
                        }

                        moved = true;
                    }
                    else if (evt.evt.GetWheelRotation() != 0)
                    {
                        camera->moveRelative(Ogre::Vector3(0, 0, evt.evt.GetWheelRotation() * MOUSE_WHEEL_FACTOR));
                        normalizeHeight();

                        moved = true;
                    }

                    if (moved)
                    {
                        updateCamera();

                        evt.vizMgr->queueRender();
                    }

                }

                void Map3DViewController::setOrientation(const Ogre::Quaternion& orientation)
                {
                    Ogre::Quaternion quat = orientation;
                    yaw_ = quat.getYaw(false).valueRadians();
                    pitch_ = quat.getPitch(false).valueRadians();

                    Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;
                    if (direction.dotProduct(Ogre::Vector3::NEGATIVE_UNIT_Z) < 0)
                    {
                        if (pitch_ > Ogre::Math::HALF_PI)
                        {
                            pitch_ = -Ogre::Math::HALF_PI + (pitch_ - Ogre::Math::HALF_PI);
                        }
                        else if (pitch_ < -Ogre::Math::HALF_PI)
                        {
                            pitch_ = Ogre::Math::HALF_PI - (-pitch_ - Ogre::Math::HALF_PI);
                        }

                        yaw_ = -yaw_;

                        if (direction.dotProduct(Ogre::Vector3::UNIT_X) < 0)
                        {
                            yaw_ -= Ogre::Math::PI;
                        }
                        else
                        {
                            yaw_ += Ogre::Math::PI;
                        }
                    }

                    normalizePitch();
                    normalizeYaw();
                }

                void Map3DViewController::lookAt(const Ogre::Vector3& point)
                {
                    camera->lookAt(point);
                    setOrientation(camera->getOrientation());
                }

                void Map3DViewController::resetView()
                {
                    camera->setPosition(POSITION_INITIAL);
                    yaw_ = YAW_INITIAL;
                    pitch_ = PITCH_INITIAL;
                    
                    updateCamera();
                }

                void Map3DViewController::setYawPitchFromOrientation(const Ogre::Quaternion& orientation)
                {
                    Ogre::Quaternion quat = orientation * ROBOT_TO_CAMERA_ROTATION.Inverse();
                    yaw_ = quat.getRoll(false).valueRadians(); // OGRE camera frame looks along -Z, so they call rotation around Z "roll".
                    pitch_ = quat.getYaw(false).valueRadians(); // OGRE camera frame has +Y as "up", so they call rotation around Y "yaw".

                    Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;

                    if (direction.dotProduct(Ogre::Vector3::NEGATIVE_UNIT_Z) < 0)
                    {
                        if (pitch_ > Ogre::Math::HALF_PI)
                        {
                            pitch_ -= Ogre::Math::PI;
                        }
                        else if (pitch_ < -Ogre::Math::HALF_PI)
                        {
                            pitch_ += Ogre::Math::PI;
                        }

                        yaw_ = -yaw_;

                        if (direction.dotProduct(Ogre::Vector3::UNIT_X) < 0)
                        {
                            yaw_ -= Ogre::Math::PI;
                        }
                        else
                        {
                            yaw_ += Ogre::Math::PI;
                        }
                    }

                    normalizePitch();
                    normalizeYaw();
                }

                void Map3DViewController::setYawPitchFromCamera()
                {
                    setYawPitchFromOrientation(camera->getOrientation());
                }

                void Map3DViewController::normalizePitch()
                {
                    if (pitch_ < PITCH_MIN)
                    {
                        pitch_ = PITCH_MIN;
                    }
                    else if (pitch_ > PITCH_MAX)
                    {
                        pitch_ = PITCH_MAX;
                    }
                }

                void Map3DViewController::normalizeYaw()
                {
                    yaw_ = fmod(yaw_, Ogre::Math::TWO_PI);

                    if (yaw_ < 0.0f)
                    {
                        yaw_ = Ogre::Math::TWO_PI + yaw_;
                    }
                }

                void Map3DViewController::normalizeHeight()
                {
                    // Ensures that the camera nevers goes under the ground
                    if (camera->getPosition().z < HEIGHT_MIN)
                    {
                        camera->setPosition(camera->getPosition().x, camera->getPosition().y, HEIGHT_MIN);
                    }
                }

                void Map3DViewController::updateCamera()
                {
                    Ogre::Quaternion pitch, yaw;

                    yaw.FromAngleAxis(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z);
                    pitch.FromAngleAxis(Ogre::Radian(pitch_), Ogre::Vector3::UNIT_Y);

                    camera->setOrientation(yaw * pitch * ROBOT_TO_CAMERA_ROTATION);

                    vizMgr->updateOCUInfoViewContent();
                }

                void Map3DViewController::yaw(float angle)
                {
                    yaw_ += angle;

                    normalizeYaw();
                }

                void Map3DViewController::pitch(float angle)
                {
                    pitch_ += angle;

                    normalizePitch();
                }

                std::string Map3DViewController::toString() const
                {
                    using namespace std;

                    std::ostringstream oss;
                    oss << "Pitch: " << pitch_ << endl;
                    oss << "Yaw: " << yaw_ << endl;
                    oss << "Camera Position: " << camera->getPosition().x << ", " << camera->getPosition().y << ", " << camera->getPosition().z << endl;
                    oss << "Camera Orientation: " << camera->getOrientation().x << ", " << camera->getOrientation().y << ", " << camera->getOrientation().z << ", " << camera->getOrientation().w << endl;
                    oss << "Camera Orientation. yaw: " << camera->getOrientation().getYaw().valueRadians() << ", pitch: " << camera->getOrientation().getPitch().valueRadians() << ", roll:" << camera->getOrientation().getRoll().valueRadians() << endl;

                    return oss.str();
                }


            }
        }
    }
}