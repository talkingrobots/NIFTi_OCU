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

#ifndef RVIZ_VISUALIZATION_FRAME_H
#define RVIZ_VISUALIZATION_FRAME_H

#include <string>
#include <vector>

#include <wx/frame.h>

#include <OGRE/OgreSceneManager.h>

#include <ros/callback_queue.h>

#include "IMultiVisualizationManager.h"

class wxAuiManager;
class wxAuiManagerEvent;
class wxTimer;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class VirtualSceneManager;

            class IVisualizationManager;

            namespace gui
            {
                class QuadVizPanel;
            }
            
            namespace tools
            {
                class Tool;
            }
        }
    }
}

namespace rviz
{
    class SplashScreen;
    class RenderPanel;

    /**
     * Main window of the software
     * @param parent
     */
    class VisualizationFrame : public wxFrame, public eu::nifti::ocu::IMultiVisualizationManager
    {
    public:
        VisualizationFrame(const wxString& frameTitle, const wxPoint& framePosition = wxDefaultPosition, const wxSize& frameSize = wxDefaultSize);
        virtual ~VisualizationFrame();

        void initialize();
        void startUpdate();

        // From IMultiVisualizationManager
        virtual void handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt, int panelNumber);
        virtual void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt, int panelNumber);
        virtual void handleSizeEvent(wxSizeEvent& evt, int pnlNumber);
        virtual void handleDragAndDropEvent(const wxString& text, int panelNum);
        
        Ogre::SceneManager* getSceneManager() const;
        void lockRender();
        void unlockRender();
        void queueRender();
        void lookAt(const Ogre::Vector3& point);

        ros::CallbackQueueInterface* getUpdateQueue();
        ros::CallbackQueueInterface* getThreadedQueue();

    protected:

       
        /**
         * Adds a tool to the collection, and to the toolbar
         * @param tool
         */
        virtual void addTool(eu::nifti::ocu::tools::Tool* tool, bool setActive = false);

        /**
         * Sets the tool that will be called by user events
         * @param tool
         */
        virtual void setActiveTool(eu::nifti::ocu::tools::Tool* tool);

        /// Called from the update timer
        void onUpdate(wxTimerEvent& event);
        void onIdle(wxIdleEvent& event);

        virtual void preInit();
        virtual void createRenderPanels();
        virtual void createSceneManagers();
        virtual void createVizualizationManagers();
        virtual void createPanels();
        virtual void createTools();
        virtual void setDefaultViewTypes();
        virtual void postInit();
        
        /**
         * Called from onUpdate(wxTimerEvent& event)
         */
        virtual void onUpdate();
        
        // wx Callbacks

        virtual void onToolClicked(wxCommandEvent& event);
   
        /**
         * This is called, in particular, when the window was minimized and gets re-opened
         * @param event
         */
        void onWindowRendered(wxAuiManagerEvent& event);

        void resetTime();

        void adjustMouseCursors();

        ros::CallbackQueue threaded_queue_;
        boost::thread_group threaded_queue_threads_;
        ros::NodeHandle update_nh_;
        ros::NodeHandle threaded_nh_;
        volatile bool shutting_down_;

        wxAuiManager* auiManager;
        Ogre::SceneManager* sceneMgr;

        eu::nifti::ocu::VirtualSceneManager* virtualSceneManager;
        
        std::vector< eu::nifti::ocu::IVisualizationManager* > vizMgrs;

        std::vector< RenderPanel* > renderPanels;

        bool disable_update_;

        wxTimer* update_timer_; ///< Update timer.  Display::update is called on each display whenever this timer fires

        ros::Time last_update_ros_time_; ///< Update stopwatch.  Stores how long it's been since the last update
        ros::WallTime last_update_wall_time_;

        std::vector< eu::nifti::ocu::tools::Tool* > tools;
        eu::nifti::ocu::tools::Tool* activeTool;
        eu::nifti::ocu::tools::Tool* defaultTool;

        eu::nifti::ocu::gui::QuadVizPanel* multiVizPanel;

        SplashScreen* splashScreen;

        static const char* SPLASH_IMAGE_PATH;
        static const char* OGRE_TEXTURES_PATH;

        // Desired number of images per second
        static const int UPDATE_FREQUENCY;

        // Maximum number of views that the user can have on the screen at once
        static const int MAX_NUM_VIEWS;

    private:
        void threadedQueueThreadFunc();

        void createColorMaterials();
        void createColorMaterial(const std::string& name, const Ogre::ColourValue& color);

    };

}

#endif // RVIZ_VISUALIZATION_FRAME_H
