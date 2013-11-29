// Benoit 2011-05-24 Comes from the RVIZ Selection Manager

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

#ifndef EU_NIFTI_OCU_SELECTION_OOI_SELECTION_MANAGER_H
#define EU_NIFTI_OCU_SELECTION_OOI_SELECTION_MANAGER_H

#include <OGRE/OgreTexture.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            class IMultiVisualizationManager;

            namespace selection
            {

                class MarkerSelectionManager
                {
                public:

                    MarkerSelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr);
                    ~MarkerSelectionManager();

                    void initialize();

                    rviz::CollObjectHandle addTrackedMarker(eu::nifti::ocu::display::IUSARMarker* marker);
                    void removeTrackedMarker(const rviz::CollObjectHandle obj);
                    
                    const eu::nifti::ocu::display::IUSARMarker* selectAnnotatedPicture(const std::string& filename);
                    const eu::nifti::ocu::display::IUSARMarker* select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);
                    void deselect();                   

                    const eu::nifti::ocu::display::IUSARMarker* getSelectedMarker();

                    eu::nifti::ocu::display::IUSARMarker* getMarker(rviz::CollObjectHandle obj);
                    
                    void focusOnSelection();

                protected:

                    eu::nifti::ocu::display::IUSARMarker* pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);
//                    rviz::Picked pick2(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);
                    
                    void addPickTechnique(rviz::CollObjectHandle handle, const Ogre::MaterialPtr& material);
                    void renderAndUnpack(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, rviz::V_Pixel& pixels);
                    void unpackColors(Ogre::Viewport* pick_viewport, Ogre::Viewport* render_viewport, const Ogre::PixelBox& box, int x1, int y1, int x2, int y2, rviz::V_Pixel& pixels);

                    eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr;

                    boost::recursive_mutex global_mutex_;

                    boost::unordered_map<rviz::CollObjectHandle, eu::nifti::ocu::display::IUSARMarker*> trackedMarkers;

                    eu::nifti::ocu::display::IUSARMarker* selectedMarker;

                    const static uint32_t s_render_texture_size_ = 1024;
                    Ogre::TexturePtr renderTexture;
                    Ogre::PixelBox pixelBox;

                    rviz::V_Pixel pixel_buffer_;
               
                    // These things deal with collision handles
                    uint32_t uid_counter_;
                    inline rviz::CollObjectHandle createHandle();
                    inline uint32_t colorToHandle(Ogre::PixelFormat format, uint32_t color);

                };
            }

        }
    }
}

#endif // EU_NIFTI_OCU_SELECTION_OOI_SELECTION_MANAGER_H
