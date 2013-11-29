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

// Benoit September 2010

#include <stdint.h>
#include <sstream>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "IVisualizationManager.h"

#include "ViewControllers/FirstPersonViewController.h"

namespace rviz
{

    const Ogre::Vector3 FirstPersonViewController::POSITION_INITIAL = Ogre::Vector3(Ogre::Vector3(0, 0, 0)); // Makes the view a little above the robot
    
    const float FirstPersonViewController::PITCH_INITIAL = Ogre::Math::HALF_PI -Ogre::Math::PI / 32; // Looking a bit below the horizon seems more natural
    // Update September 2012: With ROS Fuerte, everything was displayed vertical, so I added pi/2 in several places
    
    //const float FirstPersonViewController::YAW_INITIAL = 0.942477796; // This is 54 degree, the pan-offset due to the Point-Grey ladybug
    const float FirstPersonViewController::YAW_INITIAL = Ogre::Math::PI / 5; // This is a value found by hand, starting with Fuerte (Sept. 2012)
    
    const float FirstPersonViewController::FIELD_OF_VIEW_INITIAL = Ogre::Math::PI / 2; // This is the default if NIFTi (90 deg)

    const float FirstPersonViewController::PITCH_MIN = Ogre::Math::PI / 4;
    const float FirstPersonViewController::PITCH_MAX = Ogre::Math::PI * 3 / 4;

    const float FirstPersonViewController::PITCH_SPEED = 0.002; // How fast the camera tilts with one pixel mouse movement

    const float FirstPersonViewController::YAW_SPEED = -0.002; // How fast the camera pans with one pixel mouse movement

    const float FirstPersonViewController::ZOOM_SPEED = -0.001; // How fast the camera zooms with one move of the mouse wheel

    const float FirstPersonViewController::FIELD_OF_VIEW_MAX = Ogre::Math::HALF_PI; // pi/2 radians, 90  degrees (it's too difficult to see beyond that)
    const float FirstPersonViewController::FIELD_OF_VIEW_MIN = Ogre::Math::PI / 8; // pi/8 radians, approx. 22.5 degrees (it's zoomed in quite far already)

    FirstPersonViewController::FirstPersonViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, FrameTransformer* frameTransformer)
    : PerspectiveViewController(sceneMgr, vizMgr, camera, frameTransformer, "/omnicam")
    {
    }
    
    FirstPersonViewController::~FirstPersonViewController()
    {
    }

    void FirstPersonViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
    {
        bool moved = false;

        if (evt.evt.Dragging())
        {
            int32_t diff_x = evt.evt.GetX() - evt.lastX;
            int32_t diff_y = evt.evt.GetY() - evt.lastY;

            if (evt.evt.LeftIsDown())
            {
                yaw(diff_x * YAW_SPEED * fieldOfView);
                pitch(diff_y * PITCH_SPEED * fieldOfView);
            }

            moved = true;
        }
        else if (evt.evt.GetWheelRotation() != 0)
        {
            zoom(evt.evt.GetWheelRotation());

            moved = true;
        }

        if (moved)
        {
            updateCamera();
            
            vizMgr->updateOCUInfoViewContent();
            evt.vizMgr->queueRender();
        }

    }

    void FirstPersonViewController::setOrientation(const Ogre::Quaternion& orientation)
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

    void FirstPersonViewController::lookAt(const Ogre::Vector3& point)
    {
        camera->lookAt(point);
        setOrientation(camera->getOrientation());
    }

    void FirstPersonViewController::resetView()
    {
        camera->setPosition(POSITION_INITIAL);
        yaw_ = YAW_INITIAL;
        pitch_ = PITCH_INITIAL;
        fieldOfView = FIELD_OF_VIEW_INITIAL;
        
        updateCamera();
        
        vizMgr->updateOCUInfoViewContent();
    }

    void FirstPersonViewController::normalizePitch()
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

    void FirstPersonViewController::normalizeYaw()
    {
        yaw_ = fmod(yaw_, Ogre::Math::TWO_PI);

        if (yaw_ < 0.0f)
        {
            yaw_ = Ogre::Math::TWO_PI + yaw_;
        }
    }

    void FirstPersonViewController::normalizeFieldOfView()
    {
        if (fieldOfView > FIELD_OF_VIEW_MAX)
        {
            fieldOfView = FIELD_OF_VIEW_MAX;
        }
        else if (fieldOfView < FIELD_OF_VIEW_MIN)
        {
            fieldOfView = FIELD_OF_VIEW_MIN;
        }
    }

    void FirstPersonViewController::updateCamera()
    {
        Ogre::Matrix3 pitch, yaw;

        yaw.FromAxisAngle(Ogre::Vector3::UNIT_Z, -Ogre::Radian(yaw_));
        pitch.FromAxisAngle(Ogre::Vector3::UNIT_X, Ogre::Radian(pitch_));

        camera->setOrientation(yaw * pitch);
        camera->setFOVy(Ogre::Radian(fieldOfView));
        
        vizMgr->updateOCUInfoViewContent();
    }

    void FirstPersonViewController::yaw(float angle)
    {
        yaw_ += angle;

        normalizeYaw();
    }

    void FirstPersonViewController::pitch(float angle)
    {
        pitch_ += angle;

        normalizePitch();
    }

    void FirstPersonViewController::zoom(int wheelRotation)
    {
        float changeFactor = 1 + (wheelRotation * ZOOM_SPEED);
        fieldOfView *= changeFactor;

        normalizeFieldOfView();
    }

    void FirstPersonViewController::setPosition(float x, float y, float z)
    {
        camera->setPosition(Ogre::Vector3(x, y, z));
    }

    void FirstPersonViewController::setPitch(float pitch)
    {
        this->pitch_ = pitch;
    }

    void FirstPersonViewController::setYaw(float yaw)
    {
        this->yaw_ = yaw;
    }

    void FirstPersonViewController::setFieldOfView(float fieldOfView)
    {
        this->fieldOfView = fieldOfView;
    }

    std::string FirstPersonViewController::toString() const
    {
        using namespace std;

        std::ostringstream oss;
        oss << "Pitch: " << pitch_ << endl;
        oss << "Yaw: " << yaw_ << endl;
        oss << "Field of View: " << fieldOfView << endl;
        oss << "Camera Position: " << camera->getPosition().x << ", " << camera->getPosition().y << ", " << camera->getPosition().z << endl;

        return oss.str();
    }


}
