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

#include <wx/event.h>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>

#include "IMultiVizInputEventsHandler.h"
#include "MultiVizKeyEvent.h"
#include "MultiVizMouseEvent.h"

#include "Panels/RenderPanel.h"

namespace rviz
{

    // Passes all default parameters to wxOgreRenderWindow except the size. It
    // seems like it's a problem with GTK, but when I create a RenderPanel for
    // a camera view, only a part of the image is painted at first. For example,
    // when there is no input, "No Image" is shown. However,
    // wxDefaultSize is 20 x 20 and only this part of the "No Image" is painted.
    // If I create the RenderPanel at the size of the parent, which is certainly
    // bigger than what it will actually be once layed out, then everything is
    // painted properly.
    RenderPanel::RenderPanel(wxWindow* parent, int panelNumber, Ogre::SceneManager* sceneMgr, eu::nifti::ocu::gui::IMultiVizInputEventsHandler* evtHandler)
    : wxOgreRenderWindow(Ogre::Root::getSingletonPtr(), parent, wxID_ANY, wxDefaultPosition, parent->GetSize(), wxSUNKEN_BORDER, wxDefaultValidator, true)
    , mouse_x_(0)
    , mouse_y_(0)
    , sceneMgr(sceneMgr)
    , camera(NULL)
    , panelNumber(panelNumber)
    , evtHandler(evtHandler)
    {
        assert(panelNumber >= 0 && panelNumber <= 3);
        assert(sceneMgr != NULL);
        assert(evtHandler != NULL);

        SetFocus();
        Connect(wxEVT_CHAR, wxKeyEventHandler(RenderPanel::onKeyEvent), NULL, this);

        Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_MIDDLE_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_RIGHT_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_MOTION, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_LEFT_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_MIDDLE_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_RIGHT_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_MOUSEWHEEL, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Connect(wxEVT_LEFT_DCLICK, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
    }

    RenderPanel::~RenderPanel()
    {
        // At the end of the application, the scene manager has been destroyed, so there is no point in destroying the camera
        if(Ogre::Root::getSingletonPtr()->getSceneManagerIterator().hasMoreElements())
        {
            assert(camera != NULL);
            sceneMgr->destroyCamera(camera);
        }

        Disconnect(wxEVT_CHAR, wxKeyEventHandler(RenderPanel::onKeyEvent), NULL, this);
        Disconnect(wxEVT_LEFT_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_MIDDLE_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_RIGHT_DOWN, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_MOTION, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_LEFT_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_MIDDLE_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_RIGHT_UP, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_MOUSEWHEEL, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
        Disconnect(wxEVT_LEFT_DCLICK, wxMouseEventHandler(RenderPanel::onMouseEvent), NULL, this);
    }

    void RenderPanel::initialize()
    {
        // Creates an Ogre camera with a unique name
        std::stringstream ss;
        static int count = 0;
        ss << "RenderPanelCamera" << count++;
        camera = sceneMgr->createCamera(ss.str());

        wxOgreRenderWindow::setCamera(camera);

        // I have to do this crazy construction because wxWidgets automatically deletes the drop target. Therefore, I need to make it a dummy object, and not the render panel directly.
        RenderPanelTextDropTarget *dropTarget = new RenderPanelTextDropTarget(this);
        SetDropTarget(dropTarget);
    }

    // Todo Benoit: Try using viewports instead of 4 render panels for quad display
    //    void RenderPanel::addCameraAndViewport(std::string name)
    //    {
    //
    //    }

    void RenderPanel::onMouseEvent(wxMouseEvent& evt)
    {
        eu::nifti::ocu::gui::MultiVizMouseEvent niftiEvt(evt, mouse_x_, mouse_y_, NULL);

        mouse_x_ = evt.GetX();
        mouse_y_ = evt.GetY();

        SetFocus();

        evtHandler->handleMouseEvent(niftiEvt, panelNumber);
    }

    void RenderPanel::onKeyEvent(wxKeyEvent& evt)
    {
        eu::nifti::ocu::gui::MultiVizKeyEvent niftiEvt(evt, NULL);
        evtHandler->handleKeyEvent(niftiEvt, panelNumber);
    }

    void RenderPanel::onSize(wxSizeEvent &evt)
    {
        ogre_tools::wxOgreRenderWindow::onSize(evt);

        evtHandler->handleSizeEvent(evt, panelNumber);
    }

    bool RenderPanel::OnDropText(wxCoord x, wxCoord y, const wxString& text)
    {
        evtHandler->handleDragAndDropEvent(text, panelNumber);

        return true;
    }

    Ogre::Camera* RenderPanel::getCamera() const
    {
        return camera;
    }

    Ogre::SceneManager* RenderPanel::getSceneManager() const
    {
        return sceneMgr;
    }

    int RenderPanel::getPanelNumber() const
    {
        return panelNumber;
    }

    void RenderPanel::setPanelNumber(int panelNumber)
    {
        this->panelNumber = panelNumber;
    }

    ////////////////////////////////
    ////////////////////////////////
    //                            //
    //  RenderPanelTextDropTarget //
    //                            //
    ////////////////////////////////
    ////////////////////////////////

    RenderPanelTextDropTarget::RenderPanelTextDropTarget(RenderPanel *owner)
    :owner(owner)
    {
        
    }

    bool RenderPanelTextDropTarget::OnDropText(wxCoord x, wxCoord y, const wxString& text)
    {
        return owner->OnDropText(x, y, text);
    }

} // namespace rviz
