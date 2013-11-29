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

#ifndef RVIZ_CAMERA_DISPLAY_H
#define RVIZ_CAMERA_DISPLAY_H


#include "Displays/Display.h"
#include "Displays/ROSImageListener.h"

#include "Panels/RenderPanel.h"

#include <sensor_msgs/CameraInfo.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>



namespace Ogre
{
    class SceneNode;
    class ManualObject;
    class Rectangle2D;
    class Camera;
    class RenderWindow;
}

class wxFrame;

namespace rviz
{

    /**
     * Allows the display of a video feed
     */
    class CameraDisplay : public Display
    {
    public:
        CameraDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, Ogre::Camera* camera, Ogre::RenderWindow* renderWindow, Ogre::Viewport* viewPort);
        virtual ~CameraDisplay();

        float getAlpha()
        {
            return alpha_;
        }
        void setAlpha(float alpha);

        void setTopic(const std::string& topic);

        void setTransport(const std::string& transport);

        void setImagePosition(const std::string& image_position);

        void setZoom(float zoom);

        /**
         * Forces the display to render at the next call to update
         */
        void forceRender();

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt, float ros_dt);
        virtual void reset();

        Ogre::Image getCurrentImage();
        
        std::string getCurrentImageEncoding();
        
        //void printOutFieldOfViews();
        
        static const std::string IMAGE_POS_BACKGROUND;
        static const std::string IMAGE_POS_OVERLAY;
        static const std::string IMAGE_POS_BOTH;

    protected:

        virtual void onEnable();
        virtual void onDisable();

        void subscribe();
        void unsubscribe();

        void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

        // Updates the position of the camera based on the TFs, window size, etc.
        void updateCamera();

        /**
         * Ensures that the display shows "No Image" and that everything has been cleared
         */
        void clear();

        /**
         * Updates the status according to the number of images received
         */
        void updateStatus();

        //void onTransportEnumOptions(V_string& choices);

        Ogre::Camera* camera;
        Ogre::RenderWindow* renderWindow;
        Ogre::Viewport* viewPort;

        Ogre::SceneNode* bg_scene_node_;
        Ogre::SceneNode* fg_scene_node_;

        Ogre::Rectangle2D* bg_screen_rect_;
        Ogre::MaterialPtr bg_material_;

        Ogre::Rectangle2D* fg_screen_rect_;
        Ogre::MaterialPtr fg_material_;

        float alpha_;
        float zoom_;
        std::string transport_;
        std::string image_position_;

        message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
        tf::MessageFilter<sensor_msgs::CameraInfo> caminfo_tf_filter_;

        sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
        boost::mutex caminfo_mutex_;

        bool new_caminfo_;

        ROSImageListener imageListener;

        bool force_render_;

        class RenderListener : public Ogre::RenderTargetListener
        {
        public:
            RenderListener(CameraDisplay* display);
            virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
            virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

        private:
            CameraDisplay* display_;
        };
        RenderListener render_listener_;

    private:

    };

} // namespace rviz

#endif
