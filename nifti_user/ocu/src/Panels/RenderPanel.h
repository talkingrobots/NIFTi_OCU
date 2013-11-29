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

#ifndef RVIZ_RENDER_PANEL_H
#define RVIZ_RENDER_PANEL_H

#include <wx/dnd.h>

#include "ogre_tools/wx_ogre_render_window.h"

namespace Ogre
{
    class SceneManager;
    class Camera;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                class IMultiVizInputEventsHandler;
            }
        }
    }
}

namespace rviz
{

    class RenderPanel : public ogre_tools::wxOgreRenderWindow, public wxTextDropTarget
    {
    public:

        // panelNumber Tells which one of the four render panels this one is
        RenderPanel(wxWindow* parent, int panelNumber, Ogre::SceneManager* sceneMgr, eu::nifti::ocu::gui::IMultiVizInputEventsHandler* evtHandler);

        virtual ~RenderPanel();

        void initialize();

        virtual bool OnDropText(wxCoord x, wxCoord y, const wxString& text);

        Ogre::Camera* getCamera() const;
        Ogre::SceneManager* getSceneManager() const;

        int getPanelNumber() const;

        void setPanelNumber(int panelNumber);

    protected:
        // wx Callbacks
        /// Called when a mouse event happens inside the render window
        void onMouseEvent(wxMouseEvent& event);
        /// Called when a key is pressed
        void onKeyEvent(wxKeyEvent& event);

        void onSize(wxSizeEvent &evt);

        // Mouse handling
        int mouse_x_; ///< X position of the last mouse event
        int mouse_y_; ///< Y position of the last mouse event

        Ogre::SceneManager* sceneMgr;
        Ogre::Camera* camera;

        int panelNumber;
        eu::nifti::ocu::gui::IMultiVizInputEventsHandler* evtHandler;

    private:

    };

    // This class is used just as a workaround to wxWidgets' crazy scheme of deleting the pointer for the drop target
    class RenderPanelTextDropTarget : public wxTextDropTarget
    {
    public:
        RenderPanelTextDropTarget(RenderPanel *owner);

        virtual bool OnDropText(wxCoord x, wxCoord y, const wxString& text);

        rviz::RenderPanel *owner;
    };

} // namespace rviz

#endif

