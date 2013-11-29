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


// Benoit: 2011-05-09 ImageDisplay does not work because it draws the image
// over the scene, preventing to see anything else that the image

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>

#include <wx/frame.h>

#include "Panels/RenderPanel.h"

#include "Displays/ImageDisplay.h"

namespace rviz
{

    // Configuration for the virtual cam (from omni-cam)
    const char* ImageDisplay::CAM_VIRTUAL_TOPIC = "/viz/PTZ";
    const char* ImageDisplay::CAM_VIRTUAL_TRANSPORT = "raw";

    // Configuration for the Ladybug3 omni-cam
    const char* ImageDisplay::CAM_OMNI_TOPIC = "viz/omni";
    const char* ImageDisplay::CAM_OMNI_TRANSPORT = "theora";

    ImageDisplay::ImageDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, rviz::RenderPanel* renderPanel)
    : Display(name, sceneMgr, frameMgr, updateQueue, threadQueue)
    , transport_("raw")
    , texture(update_nh_)
    , renderPanel(renderPanel)
    {
        scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();


        static int count = 0;
        std::stringstream ss;
        ss << "ImageDisplayObject" << count++;

        screen_rect_ = new Ogre::Rectangle2D(true);
        screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material_->setSceneBlending(Ogre::SBT_REPLACE);
        material_->setDepthWriteEnabled(false);
        material_->setReceiveShadows(false);
        material_->setDepthCheckEnabled(false);

        material_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture.getTexture()->getName());
        tu->setTextureFiltering(Ogre::TFO_NONE);

        material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_->setBoundingBox(aabInf);
        screen_rect_->setMaterial(material_->getName());
        scene_node_->attachObject(screen_rect_);


        renderPanel->setAutoRender(false);
        renderPanel->getViewport()->setOverlaysEnabled(false);
        renderPanel->getViewport()->setClearEveryFrame(true);
        renderPanel->getRenderWindow()->setActive(false);
        renderPanel->getRenderWindow()->setAutoUpdated(false);
        renderPanel->getCamera()->setNearClipDistance(0.01f);
    }

    ImageDisplay::~ImageDisplay()
    {
        unsubscribe();

        delete screen_rect_;

        scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
    }

    void ImageDisplay::onEnable()
    {
        subscribe();

        renderPanel->getRenderWindow()->setActive(true);
    }

    void ImageDisplay::onDisable()
    {
        renderPanel->getRenderWindow()->setActive(false);

        unsubscribe();

        clear();
    }

    void ImageDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        texture.setTopic(topic_);
    }

    void ImageDisplay::unsubscribe()
    {
        texture.setTopic("");
    }

    void ImageDisplay::setTopic(const std::string& topic)
    {
        unsubscribe();

        topic_ = topic;
        clear();

        subscribe();
    }

    void ImageDisplay::setTransport(const std::string& transport)
    {
        transport_ = transport;

        texture.setTransportType(transport);
    }

    void ImageDisplay::setupVirtualCam()
    {
        setTopic(CAM_VIRTUAL_TOPIC);
        setTransport(CAM_VIRTUAL_TRANSPORT);
    }

    void ImageDisplay::setupOmniCam()
    {
        setTopic(CAM_OMNI_TOPIC);
        setTransport(CAM_OMNI_TRANSPORT);
    }

    void ImageDisplay::forceRender()
    {
        //force_render_ = true; Benoit Temporarily removed
    }

    void ImageDisplay::clear()
    {
        texture.clear();

        setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No image received");

        renderPanel->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
    }

    void ImageDisplay::updateStatus()
    {
        if (texture.getImageCount() == 0)
        {
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No image received");

        } else
        {
            std::stringstream ss;
            ss << texture.getImageCount() << " images received";
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }
    }

    void ImageDisplay::update(float wall_dt, float ros_dt)
    {
        //ROS_INFO("void ImageDisplay::update(float wall_dt, float ros_dt)");

        //ROS_INFO("Render panel (#%i): %i X %i", renderPanel->getPanelNumber(), renderPanel->GetSize().x, renderPanel->GetSize().y);
        //ROS_INFO("Render window: %i X %i", renderPanel->getRenderWindow()->getWidth(), renderPanel->getRenderWindow()->getHeight());
        //ROS_INFO("Texture: %i X %i", texture.getWidth(), texture.getHeight());
        

        updateStatus();

        try
        {
            texture.update();
            renderPanel->getRenderWindow()->update();
        } catch (UnsupportedImageEncoding& e)
        {
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_ERROR, e.what());
        }
    }

    void ImageDisplay::fixedFrameChanged()
    {
    }

    void ImageDisplay::reset()
    {
        Display::reset();

        clear();
    }

} // namespace rviz
