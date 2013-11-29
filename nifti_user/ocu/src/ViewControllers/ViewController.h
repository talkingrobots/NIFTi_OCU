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

#ifndef RVIZ_VIEW_CONTROLLER_H
#define RVIZ_VIEW_CONTROLLER_H

#include <string>

#include "ViewControllers/IViewController.h"

namespace Ogre
{
    class Camera;
    class SceneNode;
    class SceneManager;
    class Vector3;
    class Quaternion;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IVisualizationManager;
            
            namespace gui
            {
                class MultiVizKeyEvent;
                class MultiVizMouseEvent;
            }
        }
    }
}

namespace rviz
{
    class FrameTransformer;

    class ViewController: public eu::nifti::ocu::view::IViewController
    {
    public:
        ViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, FrameTransformer* frameTransformer, const std::string& referenceFrame);
        virtual ~ViewController();

        void activate();
        void deactivate();
        void update(float dt, float ros_dt);
        virtual const std::string& getReferenceFrameName() const;

        virtual void handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt);
        virtual void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

        virtual Ogre::Vector3 getPosition() const;
        virtual Ogre::Quaternion getOrientation() const;
        
        virtual std::string toString() const = 0;
        virtual void lookAt(const Ogre::Vector3& point) = 0;
        virtual void resetView() = 0;

    protected:
        virtual void onActivate() = 0;
        virtual void onDeactivate() = 0;

        virtual void onUpdate(float dt, float ros_dt) = 0;

        void updateReferenceNode();
        
        const std::string referenceFrame;

        Ogre::Camera* const camera;
        Ogre::SceneManager* const sceneMgr;
        Ogre::SceneNode* const referenceNode;

        eu::nifti::ocu::IVisualizationManager* const vizMgr;

        FrameTransformer* const frameTransformer;
    };

}

#endif // RVIZ_VIEW_CONTROLLER_H
