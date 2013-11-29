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

#include <wx/aui/aui.h>

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRoot.h>

#include <ogre_tools/initialization.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "Tools/Tool.h"

#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "Panels/QuadVizPanel.h"
#include "Panels/RenderPanel.h"
#include "Panels/SplashScreen.h"

#include "IVisualizationManager.h"
#include "NIFTiConstants.h"
#include "NIFTiROSUtil.h"
#include "VirtualSceneManager.h"

#include "Panels/VisualizationFrame.h"

using namespace eu::nifti::ocu;
using namespace eu::nifti::ocu::gui;

namespace rviz
{
    const char* VisualizationFrame::SPLASH_IMAGE_PATH = "/media/images/splash.png";
    const char* VisualizationFrame::OGRE_TEXTURES_PATH = "/media/OGRE/textures";

    const int VisualizationFrame::UPDATE_FREQUENCY = 30; // 30 fps

    const int VisualizationFrame::MAX_NUM_VIEWS = 4;

    VisualizationFrame::VisualizationFrame(const wxString& frameTitle, const wxPoint& framePosition, const wxSize& frameSize)
    : wxFrame(NULL, wxID_ANY, frameTitle, framePosition, frameSize, wxDEFAULT_FRAME_STYLE)
    , shutting_down_(false)
    , auiManager(NULL)
    , sceneMgr(NULL)
    , virtualSceneManager(NULL)
    , vizMgrs(std::vector< eu::nifti::ocu::IVisualizationManager* >((size_t) MAX_NUM_VIEWS, NULL)) // size_t is due to a compiler problem with STL vector
    , renderPanels(std::vector< RenderPanel* >((size_t) MAX_NUM_VIEWS, NULL)) // size_t is due to a compiler problem with STL vector
    , disable_update_(false)
    , update_timer_(NULL)
    , activeTool(NULL)
    , defaultTool(NULL)
    {
        CentreOnScreen();
        wxInitAllImageHandlers();

        threaded_nh_.setCallbackQueue(&threaded_queue_);

        for (uint32_t i = 0; i < boost::thread::hardware_concurrency(); ++i)
        {
            threaded_queue_threads_.create_thread(boost::bind(&VisualizationFrame::threadedQueueThreadFunc, this));
        }
    }

    VisualizationFrame::~VisualizationFrame()
    {
        //ROS_INFO("IN VisualizationFrame::~VisualizationFrame()");

        Disconnect(wxEVT_AUI_RENDER, wxAuiManagerEventHandler(VisualizationFrame::onWindowRendered), NULL, this);

        if (update_timer_ != NULL)
        {
            Disconnect(wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler(VisualizationFrame::onUpdate), NULL, this);
            update_timer_->Stop();
            delete update_timer_;
        }

        shutting_down_ = true;
        threaded_queue_threads_.join_all();

        for (int index = 0; index < MAX_NUM_VIEWS; index++)
        {
            // Do nothing if this view is already empty
            if (vizMgrs.at(index) != NULL)
            {
                delete vizMgrs.at(index);
            }

            //removeView(i);
        }

        std::vector< eu::nifti::ocu::tools::Tool* >::iterator tool_it = tools.begin();
        std::vector< eu::nifti::ocu::tools::Tool* >::iterator tool_end = tools.end();
        for (; tool_it != tool_end; ++tool_it)
        {
            delete *tool_it;
        }
        tools.clear();

        if (auiManager != NULL)
        {
            auiManager->UnInit();
            delete auiManager;
        }
        
        if (virtualSceneManager != NULL)
        {
            delete virtualSceneManager;
        }
        
        if(sceneMgr != NULL)
        {
            Ogre::Root::getSingletonPtr()->destroySceneManager(sceneMgr); // It seg-faults if I try to delete this pointer once it is destroyed
        }

        //ROS_INFO("OUT VisualizationFrame::~VisualizationFrame()");
    }

    void VisualizationFrame::initialize()
    {
        //ROS_INFO("IN void VisualizationFrame::initialize()");

        int index;

        ////////////////////////////////
        //
        // 1) Creates and shows a splash screen
        //
        ////////////////////////////////

        std::string splashImagePath = NIFTiConstants::ROS_PACKAGE_PATH + SPLASH_IMAGE_PATH;
        wxBitmap splash;
        splash.LoadFile(wxString::FromAscii(splashImagePath.c_str()));
        splashScreen = new SplashScreen(this, splash);
        splashScreen->Show();

        ////////////////////////////////
        //
        // 2) Initializes ROS
        //
        ////////////////////////////////

        splashScreen->setState("Initializing ROS");

        preInit();

        ////////////////////////////////
        //
        // 3) Initializes OGRE resources (and some basic GUI)
        //
        ////////////////////////////////

        splashScreen->setState("Initializing OGRE resources");

        sceneMgr = Ogre::Root::getSingletonPtr()->createSceneManager(Ogre::ST_GENERIC);

        multiVizPanel = new QuadVizPanel(this);

        // Apparently, it is not safe to initialize resources before creating
        // the RenderWindow, so one is created here.
        // http://www.ogre3d.org/forums/viewtopic.php?f=2&t=58512&start=0

        createRenderPanels();

        Ogre::Light* directional_light = sceneMgr->createLight("MainDirectional");
        directional_light->setType(Ogre::Light::LT_DIRECTIONAL);
        directional_light->setDirection(Ogre::Vector3(0, -1, 1));
        directional_light->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

        Ogre::ResourceGroupManager::getSingleton().createResourceGroup(ROS_PACKAGE_NAME);

        createColorMaterials();

        std::vector<std::string> paths;
        paths.push_back(NIFTiConstants::ROS_PACKAGE_PATH + OGRE_TEXTURES_PATH);
        ogre_tools::initializeResources(paths);

        createSceneManagers();

        createVizualizationManagers();

        ////////////////////////////////
        //
        // 4) Creates the GUI
        //
        ////////////////////////////////

        splashScreen->setState("Creating the GUI");

        auiManager = new wxAuiManager(this);

        createPanels();
        createTools();
        assert(defaultTool != NULL);
        assert(activeTool != NULL);

        auiManager->Update();

        last_update_ros_time_ = ros::Time::now();
        last_update_wall_time_ = ros::WallTime::now();

        virtualSceneManager->initialize();

        for (index = 0; index < MAX_NUM_VIEWS; index++)
        {
            if (renderPanels.at(index) != NULL)
            {
                renderPanels.at(index)->initialize();
                renderPanels.at(index)->setAutoRender(false);
                vizMgrs.at(index)->initialize();
            }
        }

        setDefaultViewTypes();

        postInit();

        Connect(wxEVT_AUI_RENDER, wxAuiManagerEventHandler(VisualizationFrame::onWindowRendered), NULL, this);

        ////////////////////////////////
        //
        // 5) Gets rid of the splash screen
        //
        ////////////////////////////////

        splashScreen->Destroy();
        splashScreen = NULL;

        startUpdate();
    }

    void VisualizationFrame::preInit()
    {

    }

    void VisualizationFrame::createRenderPanels()
    {

    }

    void VisualizationFrame::createSceneManagers()
    {

    }

    void VisualizationFrame::createVizualizationManagers()
    {

    }

    void VisualizationFrame::createPanels()
    {

    }

    void VisualizationFrame::createTools()
    {

    }

    void VisualizationFrame::setDefaultViewTypes()
    {

    }

    void VisualizationFrame::postInit()
    {

    }

    /**
     * Creates names accessible everywhere in RVIZ and that allow to always have
     * the exact same color
     * Todo Get rid of that or put it in an appropriate place
     */
    void VisualizationFrame::createColorMaterials()
    {
        createColorMaterial("RVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
        createColorMaterial("RVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f));
        createColorMaterial("RVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f));
        createColorMaterial("RVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f));
    }

    // Todo Get rid of that or put it in an appropriate place

    void VisualizationFrame::createColorMaterial(const std::string& name, const Ogre::ColourValue& color)
    {
        Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(name, ROS_PACKAGE_NAME);
        mat->setAmbient(color * 0.5f);
        mat->setDiffuse(color);
        mat->setSelfIllumination(color);
        mat->setLightingEnabled(true);
        mat->setReceiveShadows(false);
    }

    void VisualizationFrame::startUpdate()
    {
        update_timer_ = new wxTimer(this);
        update_timer_->Start(1000 / UPDATE_FREQUENCY);

        Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(VisualizationFrame::onUpdate), NULL, this);

        wxTheApp->Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(VisualizationFrame::onIdle), NULL, this);
    }

    void VisualizationFrame::onUpdate(wxTimerEvent& event)
    {
        if (disable_update_)
        {
            return;
        }

        // Ensures that we cannot come back in this method before we are done with it
        disable_update_ = true;

        onUpdate();

        ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
        ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
        float wall_dt = wall_diff.toSec();
        float ros_dt = ros_diff.toSec();

        //std::cout << "ros_dt " << ros_dt << " wall_dt " << wall_dt << std::endl;

        if (ros_dt < 0.0)
        {
            resetTime();
        }

        ros::spinOnce();

        last_update_ros_time_ = ros::Time::now();
        last_update_wall_time_ = ros::WallTime::now();

        // Updates the Frame Transformer in here
        virtualSceneManager->update(wall_dt, ros_dt);

        // Updates all viz mgrs
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->update(wall_dt, ros_dt);
            }
        }

        ros::spinOnce(); // Benoit: Why is this there twice???

        last_update_ros_time_ = ros::Time::now();
        last_update_wall_time_ = ros::WallTime::now();

        activeTool->update(wall_dt, ros_dt);

        disable_update_ = false;

        // Method from wxWidgets (yield to other apps/messages)
        wxWakeUpIdle();
    }

    void VisualizationFrame::onIdle(wxIdleEvent& evt)
    {
        virtualSceneManager->onIdle(evt);

        // Here call idle on all viz mgrs
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->onIdle(evt);
            }
        }
    }

    void VisualizationFrame::onUpdate()
    {

    }

    void VisualizationFrame::onWindowRendered(wxAuiManagerEvent& event)
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->forceReRender();
            }
        }
    }

    void VisualizationFrame::resetTime()
    {
        FrameTransformer::getTFClient()->clear();

        virtualSceneManager->resetTime();

        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->resetTime();
            }
        }
    }

    void VisualizationFrame::addTool(eu::nifti::ocu::tools::Tool* tool, bool setActive)
    {
        tools.push_back(tool);

        if (setActive)
        {
            setActiveTool(tool);
        }
    }

    void VisualizationFrame::setActiveTool(eu::nifti::ocu::tools::Tool* tool)
    {
        if (activeTool != NULL)
        {
            activeTool->deactivate();
        }

        activeTool = tool;
        activeTool->activate();

        adjustMouseCursors();
    }

    void VisualizationFrame::onToolClicked(wxCommandEvent& event)
    {
        setActiveTool(tools.at(event.GetId()));
    }

    void VisualizationFrame::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt, int panelNum)
    {
        assert(vizMgrs.at(panelNum) != NULL);

        evt.vizMgr = vizMgrs.at(panelNum);
        bool switchToDefaultTool = activeTool->handleKeyEvent(evt);

        if (switchToDefaultTool)
        {
            setActiveTool(defaultTool);
        }
    }

    void VisualizationFrame::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt, int panelNum)
    {
        assert(vizMgrs.at(panelNum) != NULL);

        evt.vizMgr = vizMgrs.at(panelNum);
        bool switchToDefaultTool = activeTool->handleMouseEvent(evt);

        if (switchToDefaultTool)
        {
            setActiveTool(defaultTool);
        }
    }

    void VisualizationFrame::handleSizeEvent(wxSizeEvent& evt, int panelNum)
    {
        assert (vizMgrs.at(panelNum) != NULL);

        vizMgrs.at(panelNum)->handleSizeEvent(evt);
    }

    void VisualizationFrame::handleDragAndDropEvent(const wxString& text, int panelNum)
    {

    }

    Ogre::SceneManager* VisualizationFrame::getSceneManager() const
    {
        return sceneMgr;
    }

    void VisualizationFrame::lockRender()
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->lockRender();
            }
        }
    }

    void VisualizationFrame::unlockRender()
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->unlockRender();
            }
        }
    }

    void VisualizationFrame::queueRender()
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->queueRender();
            }
        }
    }

    void VisualizationFrame::lookAt(const Ogre::Vector3& point)
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) != NULL)
            {
                vizMgrs.at(i)->lookAt(point);
            }
        }
    }

    ros::CallbackQueueInterface* VisualizationFrame::getUpdateQueue()
    {
        return ros::getGlobalCallbackQueue();
    }

    ros::CallbackQueueInterface* VisualizationFrame::getThreadedQueue()
    {
        return &threaded_queue_;
    }

    void VisualizationFrame::threadedQueueThreadFunc()
    {
        while (!shutting_down_)
        {
            threaded_queue_.callOne(ros::WallDuration(0.1));
        }
    }

    void VisualizationFrame::adjustMouseCursors()
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (renderPanels.at(i) == NULL) // Is this check still necessary?
            {
                break;
            }

            if (NIFTiViewsUtil::viewTypeIsCamera(vizMgrs.at(i)->getViewType()))
            {
                renderPanels.at(i)->SetCursor(activeTool->getCursorCamera());
            }
            else
            {
                renderPanels.at(i)->SetCursor(activeTool->getCursorVirtualScene());
            }

        }
    }

}
