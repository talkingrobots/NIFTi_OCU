/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


#ifndef RVIZ_VISUALIZATION_MANAGER_H_
#define RVIZ_VISUALIZATION_MANAGER_H_

#include <wx/event.h>

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/time.h>

#include "IVisualizationManager.h"



namespace Ogre
{
    class SceneManager;
}

class wxIdleEvent;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {
                class SelectionManager;
            }
        }
    }
}

namespace rviz
{

    class RenderPanel;
    class Display;
    class FrameTransformer;

    /**
     * Manages one render window (and everything that is displayed in it)
     */
    class VisualizationManager : public eu::nifti::ocu::IVisualizationManager, public wxEvtHandler
    {
    public:

        VisualizationManager(Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams);
        virtual ~VisualizationManager();

        /**
         * Returns a new viz mgr that is initialized and identical to this one
         * @param renderPanel
         * @return
         */
        virtual VisualizationManager* duplicate(RenderPanel* renderPanel) = 0;

        virtual void initialize() = 0;

        /**
         * \brief Set the coordinate frame we should be displaying in
         * @param frame The string name -- must match the frame name broadcast to libTF
         */
        virtual void setTargetFrame(const std::string& frame);
        const std::string& getTargetFrame() const;

        /**
         * \brief Set the coordinate frame we should be transforming all fixed data to
         * @param frame The string name -- must match the frame name broadcast to libTF
         */
        virtual void setFixedFrame(const std::string& frame);
        const std::string& getFixedFrame() const;

        virtual void resetTime();

        Ogre::Viewport* getViewport() const;

        eu::nifti::ocu::view::IViewController* getViewController() const;

        /**
         * Gets the view type (2D Map, 3D first person, etc.)
         * @return 
         */
        eu::nifti::ocu::ViewType getViewType() const;

        /**
         * Sets the view type (2D Map, 3D first person, etc.)
         * @return 
         */
        virtual void setViewType(eu::nifti::ocu::ViewType viewType) = 0;

        void lockRender();

        void unlockRender();

        /**
         * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
         * \note This function can be called from any thread.
         */
        void queueRender();

        /**
         * Forces the display to render at the next call to update (for now, only used for cameras - not 3D scenes)
         */
        virtual void forceReRender();

        /**
         * Makes the view point to a specific location
         */
        virtual void lookAt(const Ogre::Vector3& point);

        virtual void update(float wall_dt, float ros_dt);
        virtual void onIdle(wxIdleEvent& event);

        virtual void changeViewParams(nifti_ocu_msgs::NIFTiViewParams* viewParams);
        
        virtual void handleSizeEvent(wxSizeEvent& evt);

        const RenderPanel* getRenderPanel() const;

        virtual std::string toString() const;

    protected:

        

        Ogre::SceneManager* sceneMgr;
        eu::nifti::ocu::selection::SelectionManager* selectionMgr;

        ros::CallbackQueue* threadedQueue;

        std::string targetFrame; ///< Target coordinate frame we're displaying everything in
        bool target_frame_is_fixed_frame_;

        const RenderPanel* renderPanel;

        //std::map<std::string, Display*> displays;

        eu::nifti::ocu::view::IViewController* viewController;

        boost::mutex renderMutex;
        bool renderRequested;
        ros::WallTime timeOfLastRender;

        FrameTransformer* frameTransformer;

        eu::nifti::ocu::ViewType viewType;

        nifti_ocu_msgs::NIFTiViewParams* viewParams;
    };

}

#endif /* RVIZ_VISUALIZATION_MANAGER_H_ */
