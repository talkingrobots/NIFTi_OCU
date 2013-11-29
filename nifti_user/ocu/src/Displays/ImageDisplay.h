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

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "Displays/ROSImageTexture.h"

#include "Displays/Display.h"

namespace Ogre
{
    class SceneNode;
    class Rectangle2D;
    class Camera;
}

class wxFrame;

namespace rviz
{

    class RenderPanel;

    /**
     * \class ImageDisplay
     *
     */
    class ImageDisplay : public Display
    {
    public:
        ImageDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, rviz::RenderPanel* renderPanel);
        virtual ~ImageDisplay();

        void setTopic(const std::string& topic);

        void setTransport(const std::string& transport);

        void setupVirtualCam();
        void setupOmniCam();

        /**
         * Forces the display to render at the next call to update
         */
        void forceRender();

        // Overrides from Display
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt, float ros_dt);
        virtual void reset();

    protected:
        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        void subscribe();
        void unsubscribe();

        void clear();
        void updateStatus();

        Ogre::SceneNode* scene_node_;
        Ogre::Rectangle2D* screen_rect_;
        Ogre::MaterialPtr material_;

        std::string topic_;
        std::string transport_;

        ROSImageTexture texture;

        RenderPanel* renderPanel;

    private:

        static const float CAM_VIRTUAL_ALPHA;
        static const char* CAM_VIRTUAL_TOPIC;
        static const char* CAM_VIRTUAL_TRANSPORT;

        static const float CAM_OMNI_ALPHA;
        static const char* CAM_OMNI_TOPIC;
        static const char* CAM_OMNI_TRANSPORT;
    };

} // namespace rviz

#endif
