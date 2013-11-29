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

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>

#include <tf/transform_listener.h>

#include "nifti_ocu_msgs/NIFTiViewParams.h"

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "Displays/Display.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Panels/RenderPanel.h"

#include "ViewControllers/ViewController.h"
#include "ViewControllers/ViewControllerFactory.h"

#include "VisualizationManager.h"

namespace rviz
{

    VisualizationManager::VisualizationManager(Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selectionMgr, const RenderPanel* renderPanel, ros::CallbackQueue* threadedQueue, nifti_ocu_msgs::NIFTiViewParams* viewParams)
    : sceneMgr(sceneMgr) // Todo Benoit Move this to init
    , selectionMgr(selectionMgr) // Todo Benoit Move this to init
    , threadedQueue(threadedQueue)
    , target_frame_is_fixed_frame_(false)
    , renderPanel(renderPanel)
    , viewController(NULL)
    , renderRequested(true)
    , frameTransformer(frameTransformer)
    , viewType(eu::nifti::ocu::VIEW_TYPE_UNINITIALIZED)
    , viewParams(viewParams)
    {

    }

    VisualizationManager::~VisualizationManager()
    {
        //std::cout << "IN VisualizationManager::~VisualizationManager()" << std::endl;
        
        if( viewController != NULL )
        {
            viewController->deactivate();
            delete viewController;
        }
        
        //std::cout << "OUT VisualizationManager::~VisualizationManager()" << std::endl;
    }

    const std::string& VisualizationManager::getTargetFrame() const
    {
        if (target_frame_is_fixed_frame_)
        {
            return eu::nifti::ocu::NIFTiConstants::FIXED_FRAME_STRING;
        }

        return targetFrame;
    }

    void VisualizationManager::lockRender()
    {
        renderMutex.lock();
    }

    void VisualizationManager::unlockRender()
    {
        renderMutex.unlock();
    }

    void VisualizationManager::queueRender()
    {
        if (renderRequested == false)
        {
            wxWakeUpIdle();
        }

        renderRequested = true;
    }

    void VisualizationManager::forceReRender()
    {
        // By default, does nothing. Child classes can override.
    }

    void VisualizationManager::lookAt(const Ogre::Vector3& point)
    {
        viewController->lookAt(point);
    }

    /**
     * This call comes from the main RVIZ update loop
     * @param wall_dt Difference in wall time since the last update
     * @param ros_dt Difference in ROS time since the last update
     */
    void VisualizationManager::update(float wall_dt, float ros_dt)
    {
        // 1) Updates the frame transformer (clears the cache)
        frameTransformer->update();

        // 3) Updates the view controller
        viewController->update(wall_dt, ros_dt);

    }

    void VisualizationManager::onIdle(wxIdleEvent& evt)
    {
        ros::WallTime cur = ros::WallTime::now();
        double dt = (cur - timeOfLastRender).toSec();

        if (dt > 0.1f)
        {
            renderRequested = true;
        }

        // Cap at 60fps
        if (renderRequested == true && dt > 0.016f)
        {
            renderRequested = false;
            timeOfLastRender = cur;

            boost::mutex::scoped_lock lock(renderMutex);

            Ogre::Root::getSingletonPtr()->renderOneFrame();
        }

        evt.Skip();
    }

    void VisualizationManager::resetTime()
    {
        queueRender();
    }

    Ogre::Viewport* VisualizationManager::getViewport() const
    {
        return renderPanel->getViewport();
    }

    void VisualizationManager::setTargetFrame(const std::string& _frame)
    {
        // This seems like it does not make sense anymore
        //assert(viewController != NULL);

        target_frame_is_fixed_frame_ = false;
        std::string frame = _frame;
        if (frame == eu::nifti::ocu::NIFTiConstants::FIXED_FRAME_STRING)
        {
            frame = getFixedFrame();
            target_frame_is_fixed_frame_ = true;
        }

        std::string remapped_name = frameTransformer->getTFClient()->resolve(frame);

        if (targetFrame == remapped_name)
        {
            return;
        }

        targetFrame = remapped_name;
    }

    void VisualizationManager::setFixedFrame(const std::string& frame)
    {
        frameTransformer->setFixedFrame(frame);

        if (target_frame_is_fixed_frame_)
        {
            setTargetFrame(eu::nifti::ocu::NIFTiConstants::FIXED_FRAME_STRING);
        }
    }

    const std::string& VisualizationManager::getFixedFrame() const
    {
        return frameTransformer->getFixedFrame();
    }

    eu::nifti::ocu::view::IViewController* VisualizationManager::getViewController() const
    {
        return viewController;
    }

    eu::nifti::ocu::ViewType VisualizationManager::getViewType() const
    {
        return viewType;
    }

    void VisualizationManager::changeViewParams(nifti_ocu_msgs::NIFTiViewParams* viewParams)
    {
        this->viewParams = viewParams;
    }
    
    void VisualizationManager::handleSizeEvent(wxSizeEvent& evt)
    {
        
    }

    const RenderPanel* VisualizationManager::getRenderPanel() const
    {
        return renderPanel;
    }

    std::string VisualizationManager::toString() const
    {
        using namespace std;

        stringstream ss;

        ss << "Fixed Frame: " << getFixedFrame() << endl;
        ss << "Target Frame: " << getTargetFrame() << endl;
        ss << "target_frame_is_fixed_frame_: " << target_frame_is_fixed_frame_ << endl;
        ss << "View Type: " << viewType << endl;
        ss << "Frame Transformer: " << &frameTransformer << endl;
        ss << "Render Panel #" << renderPanel->getPanelNumber() << ": " << &renderPanel << " (" << renderPanel->GetSize().GetWidth() << " x " << renderPanel->GetSize().GetHeight() << ")" << endl;
        ss << "Render Window: " << renderPanel->getRenderWindow()->getWidth() << " x " << renderPanel->getRenderWindow()->getHeight() << endl;
        ss << "Render Viewport: " << renderPanel->getViewport()->getActualWidth() << " x " << renderPanel->getViewport()->getActualHeight() << endl;
        ss << "Render Viewport Overlays Enabled: " << renderPanel->getViewport()->getOverlaysEnabled() << endl;
        ss << "Render Viewport ClearEveryFrame: " << renderPanel->getViewport()->getClearEveryFrame() << endl;
        ss << "Render Window Active: " << renderPanel->getRenderWindow()->isActive() << endl;
        ss << "Render Window AutoUpdated: " << renderPanel->getRenderWindow()->isAutoUpdated() << endl;
        ss << "Camera Position: " << renderPanel->getCamera()->getPosition().x << ", " << renderPanel->getCamera()->getPosition().y << ", " << renderPanel->getCamera()->getPosition().z << endl;
        ss << "Camera Real Direction: " << renderPanel->getCamera()->getRealDirection().x << ", " << renderPanel->getCamera()->getRealDirection().y << ", " << renderPanel->getCamera()->getRealDirection().z << endl;

        Ogre::Quaternion q = renderPanel->getCamera()->getRealOrientation();
        ss << "Camera Real Orientation (wxyz): " << q.w << ", " << q.x << ", " << q.y << ", " << q.z << endl;


//        // http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Quaternion+and+Rotation+Primer
//        
//        Ogre::Quaternion rotX(sqrt(0.5), sqrt(0.5), 0, 0);
//        Ogre::Quaternion rotXNeg(sqrt(0.5), -sqrt(0.5), 0, 0);
//        //sqrt(0.5)	sqrt(0.5)	0	0	90° rotation around X axis 
//
//        Ogre::Quaternion rotY(sqrt(0.5), 0, sqrt(0.5), 0);
//        Ogre::Quaternion rotYNeg(sqrt(0.5), 0, -sqrt(0.5), 0);
//        //sqrt(0.5)	0	sqrt(0.5)	0	90° rotation around Y axis 
//
//        Ogre::Quaternion rotZ(sqrt(0.5), 0, 0, sqrt(0.5));
//        //sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis 
//
//        q = q * rotXNeg * rotZ;
//
//        ss << "... turned: " << q.w << ", " << q.x << ", " << q.y << ", " << q.z << endl;


        //ss << "Total number of cameras: " << sceneMgr->getCameras().size() << endl;

        //ss << "Map: " << displays.at("Map")->getStatus("Map").msg << endl;

        // Benoit todo Print status about all displays

        return ss.str();
    }

} // namespace rviz
