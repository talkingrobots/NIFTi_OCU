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

#ifndef RVIZ_FPS_VIEW_CONTROLLER_H
#define RVIZ_FPS_VIEW_CONTROLLER_H

#include "ViewControllers/PerspectiveViewController.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

    /**
     * \class FPSCamera
     * \brief A first-person camera, controlled by yaw, pitch, and position.
     *  Pitch (up down), yaw (turn left, right)
     */
    class FirstPersonViewController : public eu::nifti::ocu::view::PerspectiveViewController
    {
    public:
        FirstPersonViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, FrameTransformer* frameTransformer);
        virtual ~FirstPersonViewController();

        void yaw(float angle);
        void pitch(float angle);
        void zoom(int wheelRotation);
        void setPosition(float x, float y, float z);
        void setPitch(float pitch);
        void setYaw(float yaw);
        void setFieldOfView(float fieldOfView);

        void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

        std::string toString() const;

        void lookAt(const Ogre::Vector3& point);
        void resetView();

       


    protected:
        void setOrientation(const Ogre::Quaternion& orientation);

        /**
         * \brief Normalizes the camera's pitch, preventing it from reaching vertical (or turning upside down)
         */
        void normalizePitch();
        /**
         * \brief Normalizes the camera's yaw in the range [0, 2*pi)
         */
        void normalizeYaw();

        void normalizeFieldOfView();

        void updateCamera();

        float yaw_; ///< The camera's yaw (rotation around the y-axis), in radians
        float pitch_; ///< The camera's pitch (rotation around the x-axis), in radians

        // Field of view of the camera, in radians
        float fieldOfView;

        static const Ogre::Vector3 POSITION_INITIAL;
        static const float PITCH_INITIAL;
        static const float YAW_INITIAL;
        static const float FIELD_OF_VIEW_INITIAL;

        static const float PITCH_MIN;
        static const float PITCH_MAX;

        static const float PITCH_SPEED;

        static const float YAW_SPEED;

        static const float ZOOM_SPEED;

        static const float FIELD_OF_VIEW_MAX;
        static const float FIELD_OF_VIEW_MIN;
    };

}

#endif // RVIZ_FPS_VIEW_CONTROLLER_H
