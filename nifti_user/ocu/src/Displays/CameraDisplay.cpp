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


#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>

#include <ogre_tools/axes.h>

#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "FloatValidator.h"

#include "Displays/CameraDisplay.h"

namespace rviz
{

    const std::string CameraDisplay::IMAGE_POS_BACKGROUND = "background";
    const std::string CameraDisplay::IMAGE_POS_OVERLAY = "overlay";
    const std::string CameraDisplay::IMAGE_POS_BOTH = "background & overlay";

    CameraDisplay::CameraDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, Ogre::Camera* camera, Ogre::RenderWindow* renderWindow, Ogre::Viewport* viewPort)
    : Display(name, sceneMgr, frameMgr, updateQueue, threadQueue)
    , camera(camera)
    , renderWindow(renderWindow)
    , viewPort(viewPort)
    , zoom_(1)
    , transport_("raw")
    , image_position_(IMAGE_POS_OVERLAY)
    , caminfo_tf_filter_(*frameMgr->getTFClient(), "", 2, update_nh_)
    , new_caminfo_(false)
    , imageListener(update_nh_)
    , force_render_(false)
    , render_listener_(this)
    {
        bg_scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();
        fg_scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();

        {
            static int count = 0;
            std::stringstream ss;
            ss << "CameraDisplayObject" << count++;

            //background rectangle
            bg_screen_rect_ = new Ogre::Rectangle2D(true);
            bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

            ss << "Material";
            bg_material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            bg_material_->setDepthWriteEnabled(false);

            bg_material_->setReceiveShadows(false);
            bg_material_->setDepthCheckEnabled(false);

            bg_material_->getTechnique(0)->setLightingEnabled(false);
            Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
            tu->setTextureName(imageListener.getTexture()->getName());
            tu->setTextureFiltering(Ogre::TFO_NONE);
            tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);

            bg_material_->setCullingMode(Ogre::CULL_NONE);
            bg_material_->setSceneBlending(Ogre::SBT_REPLACE);

            Ogre::AxisAlignedBox aabInf;
            aabInf.setInfinite();

            bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
            bg_screen_rect_->setBoundingBox(aabInf);
            bg_screen_rect_->setMaterial(bg_material_->getName());

            bg_scene_node_->attachObject(bg_screen_rect_);
            bg_scene_node_->setVisible(false);

            //overlay rectangle
            fg_screen_rect_ = new Ogre::Rectangle2D(true);
            fg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

            fg_material_ = bg_material_->clone(ss.str() + "fg");
            fg_screen_rect_->setBoundingBox(aabInf);
            fg_screen_rect_->setMaterial(fg_material_->getName());

            fg_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

            fg_scene_node_->attachObject(fg_screen_rect_);
            fg_scene_node_->setVisible(false);
        }

        // From electric. Seems to crash when I do overlays without calling that first
        setAlpha(1.0f);

        renderWindow->addListener(&render_listener_);
        viewPort->setOverlaysEnabled(false);
        viewPort->setClearEveryFrame(true);
        renderWindow->setActive(false);
        renderWindow->setAutoUpdated(false);
        camera->setNearClipDistance(0.1f);

        caminfo_tf_filter_.connectInput(caminfo_sub_);
        caminfo_tf_filter_.registerCallback(boost::bind(&CameraDisplay::caminfoCallback, this, _1));
        frameMgr->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);
    }

    CameraDisplay::~CameraDisplay()
    {
        unsubscribe();
        caminfo_tf_filter_.clear();

        delete bg_screen_rect_;
        delete fg_screen_rect_;

        bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_->getName());
        fg_scene_node_->getParentSceneNode()->removeAndDestroyChild(fg_scene_node_->getName());

        // Benoit: I added this to undo what it does in the constructor
        viewPort->setOverlaysEnabled(true);
        renderWindow->setAutoUpdated(true);
        renderWindow->removeListener(&render_listener_);
    }

    void CameraDisplay::onEnable()
    {
        subscribe();

        renderWindow->setActive(true);
    }

    void CameraDisplay::onDisable()
    {
        renderWindow->setActive(false);

        unsubscribe();

        clear();
    }

    void CameraDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        try
        {
            imageListener.setTopic(topic);
        }
        catch (image_transport::TransportLoadException& ex)
        {
            std::cerr << ex.what() << std::endl;
            unsubscribe();
            throw;
        }

        // parse out the namespace from the topic so we can subscribe to the caminfo
        std::string caminfo_topic = "camera_info";
        size_t pos = topic.rfind('/');
        if (pos != std::string::npos)
        {
            std::string ns = topic;
            ns.erase(pos);

            caminfo_topic = ns + "/" + caminfo_topic;
        }

        caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
    }

    void CameraDisplay::unsubscribe()
    {
        imageListener.setTopic("");
        caminfo_sub_.unsubscribe();
    }

    void CameraDisplay::setAlpha(float alpha)
    {
        if (this->alpha_ == alpha)
            return;

        alpha_ = alpha;

        // Below from electric
        Ogre::Pass* pass = fg_material_->getTechnique(0)->getPass(0);
        if (pass->getNumTextureUnitStates() > 0)
        {
            Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
            tex_unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_);
        }
        else
        {
            fg_material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
            fg_material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
        }

        force_render_ = true;
        causeRender();
    }

    void CameraDisplay::setZoom(float zoom)
    {
        if (fabs(zoom) < .00001 || fabs(zoom) > 100000)
        {
            return;
        }
        zoom_ = zoom;

        force_render_ = true;
        causeRender();
    }

    void CameraDisplay::setTopic(const std::string& topic)
    {
        if (this->topic == topic)
            return;

        unsubscribe();

        this->topic = topic;
        clear();

        subscribe();
    }

    void CameraDisplay::setTransport(const std::string& transport)
    {
        if (this->transport_ == transport)
            return;

        transport_ = transport;

        try
        {
            imageListener.setTransportType(transport);
        }
        catch (image_transport::TransportLoadException& ex)
        {
            std::cerr << ex.what() << std::endl;
            unsubscribe();
            throw;
        }
    }

    void CameraDisplay::setImagePosition(const std::string& image_position)
    {
        image_position_ = image_position;

        force_render_ = true;
        causeRender();
    }

    void CameraDisplay::forceRender()
    {
        force_render_ = true;
    }

    void CameraDisplay::clear()
    {
        imageListener.clear();
        force_render_ = true;

        new_caminfo_ = false;
        current_caminfo_.reset();

        setStatus("CameraInfo", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No CameraInfo received on [" + caminfo_sub_.getTopic() + "].  Topic may not exist.");
        setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No image received");

        camera->setPosition(Ogre::Vector3(999999, 999999, 999999));
    }

    void CameraDisplay::updateStatus()
    {
        if (imageListener.getImageCount() == 0)
        {
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No image received");
        }
        else
        {
            std::stringstream ss;
            ss << imageListener.getImageCount() << " images received";
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }
    }

    void CameraDisplay::update(float wall_dt, float ros_dt)
    {
        updateStatus();

        try
        {
            if (imageListener.update() || force_render_)
            {
                float old_alpha = alpha_;
                if (imageListener.getImageCount() == 0)
                {
                    alpha_ = 1.0f;
                }

                updateCamera();
                renderWindow->update();
                alpha_ = old_alpha;

                force_render_ = false;
            }
        }
        catch (UnsupportedImageEncoding& e)
        {
            setStatus("Image", eu::nifti::ocu::STATUS_LEVEL_ERROR, e.what());
        }
    }

    void CameraDisplay::updateCamera()
    {
        sensor_msgs::CameraInfo::ConstPtr info;
        sensor_msgs::Image::ConstPtr image;
        {
            boost::mutex::scoped_lock lock(caminfo_mutex_);

            info = current_caminfo_;
            image = imageListener.getLatestImageMsgReceived();
        }

        if (!info || !image)
        {
            return;
        }

        // To hide / mask part of the image (omni_cam)

        //convert image to cv_image (cv_bridge)

        //crop cv_image with ROI (cvSetROI)

        //convert cv_image back to sensor_msgs:Image

        // OR

        // Add a mask (tinted window) on the sides (cvadd opencv)

        if (!FloatValidator::validateFloats(*info))
        {
            setStatus("CameraInfo", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Contains invalid floating point values (nans or infs)");
            return;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        frameTransformer->getTransform(image->header, position, orientation);

        // convert vision (Z-forward) frame to ogre frame (Z-out)
        orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

        float img_width = info->width;
        float img_height = info->height;

        // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
        if (img_width == 0)
        {
            ROS_DEBUG("Malformed CameraInfo on camera [%s], width = 0", getName().c_str());

            img_width = imageListener.getTexture().get()->getWidth(); // Benoit: uncertain if this is correct
        }

        if (img_height == 0)
        {
            ROS_DEBUG("Malformed CameraInfo on camera [%s], height = 0", getName().c_str());

            img_height = imageListener.getTexture().get()->getHeight(); // Benoit: uncertain if this is correct
        }

        if (img_height == 0.0 || img_width == 0.0)
        {
            setStatus("CameraInfo", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
            return;
        }

        double fx = info->P[0];
        double fy = info->P[5];


        float win_width = camera->getViewport()->getActualWidth();
        float win_height = camera->getViewport()->getActualHeight();
        float zoom_x = zoom_;
        float zoom_y = zoom_;

        //preserve aspect ratio
        if (win_width != 0 && win_height != 0)
        {
            float img_aspect = (img_width / fx) / (img_height / fy);
            float win_aspect = win_width / win_height;

            if (img_aspect > win_aspect)
            {
                zoom_y = zoom_y / img_aspect * win_aspect;
            }
            else
            {
                zoom_x = zoom_x / win_aspect * img_aspect;
            }
        }

        // Add the camera's translation relative to the left camera (from P[3]);
        double tx = -1 * (info->P[3] / fx);
        Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
        position = position + (right * tx);

        double ty = -1 * (info->P[7] / fy);
        Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
        position = position + (down * ty);

        if (!FloatValidator::validateFloats(position))
        {
            setStatus("CameraInfo", eu::nifti::ocu::STATUS_LEVEL_ERROR, "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
            return;
        }

        camera->setPosition(position);
        camera->setOrientation(orientation);

        // calculate the projection matrix
        double cx = info->P[2];
        double cy = info->P[6];

        double far_plane = 100;
        double near_plane = 0.01;

        Ogre::Matrix4 proj_matrix;
        proj_matrix = Ogre::Matrix4::ZERO;

        proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x;
        proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y;

        proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x;
        proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y;

        proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
        proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

        proj_matrix[3][2] = -1;

        camera->setCustomProjectionMatrix(true, proj_matrix);

        setStatus("CameraInfo", eu::nifti::ocu::STATUS_LEVEL_OK, "");

        //adjust the image rectangles to fit the zoom & aspect ratio
        bg_screen_rect_->setCorners(-1.0f * zoom_x, 1.0f * zoom_y, 1.0f * zoom_x, -1.0f * zoom_y);
        fg_screen_rect_->setCorners(-1.0f * zoom_x, 1.0f * zoom_y, 1.0f * zoom_x, -1.0f * zoom_y);

        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        bg_screen_rect_->setBoundingBox(aabInf);
        fg_screen_rect_->setBoundingBox(aabInf);


    }

    void CameraDisplay::caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);
        current_caminfo_ = msg;
        new_caminfo_ = true;
    }

    void CameraDisplay::fixedFrameChanged()
    {
        caminfo_tf_filter_.setTargetFrame(fixed_frame_);

        try
        {
            imageListener.setFrame(fixed_frame_, frameTransformer->getTFClient());
        }
        catch (image_transport::TransportLoadException& ex)
        {
            std::cerr << ex.what() << std::endl;
            unsubscribe();
            throw;
        }
    }

    void CameraDisplay::reset()
    {
        Display::reset();

        clear();
    }

    Ogre::Image CameraDisplay::getCurrentImage()
    {
        //return imageListener.getOgreImageFromTexture();
        return imageListener.getLatestImageLoaded();
    }

    std::string CameraDisplay::getCurrentImageEncoding()
    {
        if(imageListener.getLatestImageMsgReceived())
        {
            return imageListener.getLatestImageMsgReceived()->encoding;
        }
        else // if there is no image received yet, return the encoding of the default image
        {
            return sensor_msgs::image_encodings::RGBA8;
        }
        
    }

//    void CameraDisplay::printOutFieldOfViews()
//    {
//        if (!current_caminfo_ || !imageListener.getLatestImageMsgReceived())
//        {
//            ROS_INFO_STREAM("H: -1 V: -1");
//        }
//        else
//        {
//            double fov_horizontal = 2.0 * atan(current_caminfo_->K.at(2) / current_caminfo_->K.at(0));
//            double fov_vertical = 2.0 * atan(current_caminfo_->K.at(5) / current_caminfo_->K.at(4));
//
//            ROS_INFO_STREAM("H: " << fov_horizontal << " V: " << fov_vertical);
//        }
//    }

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    CameraDisplay::RenderListener::RenderListener(CameraDisplay* display)
    : display_(display)
    {

    }

    void CameraDisplay::RenderListener::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
    {
        // These two lines are from electric
        display_->bg_scene_node_->setVisible(display_->image_position_ == IMAGE_POS_BACKGROUND || display_->image_position_ == IMAGE_POS_BOTH);
        display_->fg_scene_node_->setVisible(display_->image_position_ == IMAGE_POS_OVERLAY || display_->image_position_ == IMAGE_POS_BOTH);
    }

    void CameraDisplay::RenderListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
    {
        // These two lines are from electric
        display_->bg_scene_node_->setVisible(false);
        display_->fg_scene_node_->setVisible(false);
    }




} // namespace rviz
