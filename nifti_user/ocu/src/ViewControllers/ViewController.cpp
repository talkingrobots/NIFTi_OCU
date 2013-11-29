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


#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include "IVisualizationManager.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "ViewControllers/ViewController.h"

namespace rviz
{

    ViewController::ViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, FrameTransformer* frameTransformer, const std::string& referenceFrame)
    : referenceFrame(referenceFrame)
    , camera(camera)
    , sceneMgr(sceneMgr)
    , referenceNode(sceneMgr->getRootSceneNode()->createChildSceneNode())
    , vizMgr(vizMgr)
    , frameTransformer(frameTransformer)
    {
        assert(vizMgr != NULL);
    }

    ViewController::~ViewController()
    {
        sceneMgr->destroySceneNode(referenceNode);
    }

    const std::string& ViewController::getReferenceFrameName() const
    {
        return referenceFrame;

    }

    void ViewController::activate()
    {
        updateReferenceNode();

        onActivate();
    }

    void ViewController::deactivate()
    {
        onDeactivate();

        //this->camera = 0; //Benoit 2012-06-28 This seems useless, because the render panel will destroy the camera in its destructor
    }

    void ViewController::update(float dt, float ros_dt)
    {
        updateReferenceNode();
        onUpdate(dt, ros_dt);
    }

    void ViewController::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
    {

    }

    void ViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
    {

    }

    Ogre::Vector3 ViewController::getPosition() const
    {
        return camera->getRealPosition();
    }

    Ogre::Quaternion ViewController::getOrientation() const
    {
        return camera->getOrientation();
    }

    void ViewController::updateReferenceNode()
    {
        assert(referenceNode != NULL);
        assert(frameTransformer != NULL);

        Ogre::Vector3 old_position = referenceNode->getPosition();
        Ogre::Quaternion old_orientation = referenceNode->getOrientation();
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (frameTransformer->getTransform(referenceFrame, ros::Time(), position, orientation))
        {
            referenceNode->setPosition(position);
            referenceNode->setOrientation(orientation);

            //printf("ViewController. Ref frame: %s. x %f y %f z %f\n", referenceFrame.c_str(), referenceNode->getPosition().x, referenceNode->getPosition().y, referenceNode->getPosition().z);

            if (!old_position.positionEquals(position, 0.01) ||
                    !old_orientation.equals(orientation, Ogre::Radian(0.05)))
            {
                vizMgr->queueRender();
            }
        }
    }

}
