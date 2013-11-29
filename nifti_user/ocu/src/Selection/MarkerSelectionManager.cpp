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

#include <boost/foreach.hpp>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreTextureManager.h>
//#include <OGRE/OgrePlaneBoundedVolume.h>

#include "IMultiVisualizationManager.h"

#include "Displays/Markers/InFieldPictureMarker.h"
#include "Displays/Markers/ObjectOfInterestMarker.h"

#include "Selection/MarkerSelectionManager.h"

using namespace eu::nifti::ocu::display;
using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {

                MarkerSelectionManager::MarkerSelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr)
                : multiVizMgr(multiVizMgr)
                , selectedMarker(NULL)
                , uid_counter_(0)
                {
                    assert(multiVizMgr != NULL);

                    pixelBox.data = 0;
                }

                MarkerSelectionManager::~MarkerSelectionManager()
                {
                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    deselect();

                    trackedMarkers.clear();

                    delete [] (uint8_t*) pixelBox.data;
                }

                // From rviz::SelectionHandler

                inline uint32_t MarkerSelectionManager::colorToHandle(Ogre::PixelFormat format, uint32_t color)
                {
                    uint32_t handle = 0;
                    if (format == Ogre::PF_A8R8G8B8 || format == Ogre::PF_X8R8G8B8)
                    {
                        handle = color & 0x00ffffff;
                    }
                    else if (format == Ogre::PF_R8G8B8A8)
                    {
                        handle = color >> 8;
                    }
                    else
                    {
                        ROS_DEBUG("Incompatible pixel format [%d]", format);
                    }

                    return handle;
                }

                inline rviz::CollObjectHandle MarkerSelectionManager::createHandle()
                {
                    if (uid_counter_ > 0x00ffffff)
                    {
                        uid_counter_ = 0;
                    }

                    uint32_t handle = 0;

                    do
                    {
                        handle = (++uid_counter_) << 4;
                        handle ^= 0x00707070;
                        handle &= 0x00ffffff;
                    }
                    while (trackedMarkers.find(handle) != trackedMarkers.end());

                    return handle;
                }

                void MarkerSelectionManager::initialize()
                {
                    // Create our render texture
                    renderTexture = Ogre::TextureManager::getSingleton().createManual("SelectionTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, s_render_texture_size_, s_render_texture_size_, 0, Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);
                    Ogre::RenderTexture* render_texture = renderTexture->getBuffer()->getRenderTarget();
                    render_texture->setAutoUpdated(false);
                }

                /**
                 * This method is theoretically slow because it linearly goes through the list to find the item, instead
                 * of using a map or something similar. For now it's ok though.
                 * @param filename
                 * @return 
                 */
                const eu::nifti::ocu::display::IUSARMarker* MarkerSelectionManager::selectAnnotatedPicture(const std::string& filename)
                {
                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    deselect();
                
                    // This typedef is necessary otherwise the BOOST_FOREACH loop does not compile
                    typedef boost::unordered_map<rviz::CollObjectHandle, eu::nifti::ocu::display::IUSARMarker*> MapOfCollisionHandleToMarker;
                    BOOST_FOREACH(const MapOfCollisionHandleToMarker::value_type& pair, trackedMarkers)
                    {
                        eu::nifti::ocu::display::IUSARMarker* marker = pair.second;

                        // Skips the markers other than in-field pictures
                        if (marker->getMarkerType() != eu::nifti::ocu::display::IUSARMarker::ANNOTATED_PICTURE)
                            continue;

                        //const eu::nifti::ocu::display::AnnotatedPictureMarker* picMarker = (const eu::nifti::ocu::display::AnnotatedPictureMarker*) pair.second;

                        if(marker->getIdentifier() == filename)
                        {
                            selectedMarker = marker;
                            selectedMarker->onSelect();
                            break;
                        }
                    }
                    
                    return selectedMarker;
                }

                const eu::nifti::ocu::display::IUSARMarker* MarkerSelectionManager::select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
                {
                    //ROS_INFO("IN void ObjectOfInterestSelectionManager::select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2): %i %i %i %i", x1, y1, x2, y2);

                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    multiVizMgr->lockRender();

                    deselect();

                    selectedMarker = pick(viewport, x1, y1, x2, y2);

                    //pick2(viewport, x1, y1, x2, y2);

                    //std::cout << "The following was picked: " << selection.handle << std::endl;

                    if (selectedMarker != NULL)
                    {
                        selectedMarker->onSelect();
                    }

                    multiVizMgr->unlockRender();

                    //ROS_INFO("OUT void ObjectOfInterestSelectionManager::select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)");

                    return selectedMarker;
                }

                void MarkerSelectionManager::unpackColors(Ogre::Viewport* pick_viewport, Ogre::Viewport* render_viewport, const Ogre::PixelBox& box, int x1, int y1, int x2, int y2, rviz::V_Pixel& pixels)
                {
                    float width = pick_viewport->getActualWidth();
                    float height = pick_viewport->getActualHeight();
                    int render_width = render_viewport->getActualWidth();
                    int render_height = render_viewport->getActualHeight();



                    float fracw = render_width / width;
                    float frach = render_height / height;
                    x1 = x1 * fracw;
                    x2 = x2 * fracw;
                    y1 = y1 * frach;
                    y2 = y2 * frach;

                    x1 = x1 < 0 ? 0 : (x1 > render_width ? render_width : x1);
                    y1 = y1 < 0 ? 0 : (y1 > render_height ? render_height : y1);
                    x2 = x2 < 0 ? 0 : (x2 > render_width ? render_width : x2);
                    y2 = y2 < 0 ? 0 : (y2 > render_height ? render_height : y2);

                    int step_x = (x2 - x1) >= 0 ? 1 : -1;
                    int step_y = (y2 - y1) >= 0 ? 1 : -1;
                    pixels.resize((abs(x2 - x1) + 1) * (abs(y2 - y1) + 1));
                    int i = 0;

                    // Goes through all pixels, doing the 1st row, then the 2nd, etc.
                    for (int y = y1; y != (y2 + step_y); y += step_y)
                    {
                        for (int x = x1; x != (x2 + step_x); x += step_x)
                        {
                            uint32_t pos = (x + y * render_width) * 4;

                            uint32_t pix_col_val = *(uint32_t*) ((uint8_t*) box.data + pos);
                            uint32_t handle = colorToHandle(box.format, pix_col_val);

                            rviz::Pixel& p = pixels[i];
                            p.x = x;
                            p.y = y;
                            p.handle = handle;

                            //if (handle != 0)
                            //    ROS_INFO("Unpacking colors for pixel (%i, %i): %i", x, y, handle);

                            ++i;
                        }
                    }
                }

                void MarkerSelectionManager::renderAndUnpack(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, rviz::V_Pixel& pixels)
                {
                    Ogre::TexturePtr tex = renderTexture;
                    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
                    Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();

                    if (render_texture->getNumViewports() == 0 || render_texture->getViewport(0)->getCamera() != viewport->getCamera())
                    {
                        render_texture->removeAllViewports();
                        render_texture->addViewport(viewport->getCamera());
                        Ogre::Viewport* render_viewport = render_texture->getViewport(0);
                        render_viewport->setClearEveryFrame(true);
                        render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
                        render_viewport->setOverlaysEnabled(false);
                        render_viewport->setMaterialScheme("Pick");
                    }

                    render_texture->update();

                    Ogre::Viewport* render_viewport = pixel_buffer->getRenderTarget()->getViewport(0);
                    int render_width = render_viewport->getActualWidth();
                    int render_height = render_viewport->getActualHeight();

                    Ogre::PixelFormat format = pixel_buffer->getFormat();

                    int size = Ogre::PixelUtil::getMemorySize(render_width, render_height, 1, format);
                    uint8_t* data = new uint8_t[size];

                    delete [] (uint8_t*) pixelBox.data;
                    pixelBox = Ogre::PixelBox(render_width, render_height, 1, format, data);

                    pixel_buffer->blitToMemory(pixelBox);

                    unpackColors(viewport, render_viewport, pixelBox, x1, y1, x2, y2, pixels);
                }

                // Does the initial object picking, determines which object has been selected

                eu::nifti::ocu::display::IUSARMarker* MarkerSelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
                {
                    //ROS_INFO("IN rviz::Picked ObjectOfInterestSelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, rviz::M_Picked& results)");

                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    rviz::V_Pixel& pixels = pixel_buffer_;
                    renderAndUnpack(viewport, x1, y1, x2, y2, pixels);

                    rviz::Picked selection(0);

                    // Goes through all pixels in the selection area, and tries to get an entity  
                    // If more than one is found within the range of pixels, the first one found will be selected
                    rviz::V_Pixel::iterator it = pixels.begin();
                    rviz::V_Pixel::iterator end = pixels.end();
                    for (; it != end; ++it)
                    {
                        const rviz::Pixel& p = *it;
                        rviz::CollObjectHandle handle = p.handle;

                        //std::cout << "Pixel " << p.x << ", " << p.y  << ": handle #" << handle << std::endl;

                        if (handle != 0)
                        {
                            //std::cout << "Found a collision handle at pixel " << p.x << ", " << p.y << ": handle #" << handle << std::endl;

                            // An entity handle was found (i.e. an object is at this pixel)
                            eu::nifti::ocu::display::IUSARMarker* marker = getMarker(handle);
                            if (marker != NULL)
                            {
                                selectedMarker = marker;
                                break;
                            }
                            else
                            {
                                // This seems normal in the way implemented in RVIZ
                                //ROS_ERROR("There is no marker for this handle: %i", handle);
                            }
                        }
                    }

                    //if (selection.handle == 0)
                    //    std::cout << "Found no collision handles" << std::endl;


                    // Reset the "last viewport" of the camera, since picking changes it.
                    viewport->getCamera()->_notifyViewport(viewport);

                    //ROS_INFO("OUT rviz::Picked ObjectOfInterestSelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, rviz::M_Picked& results)");

                    return selectedMarker;
                }

                //                rviz::Picked ObjectOfInterestSelectionManager::pick2(Ogre::Viewport* viewport, int left, int bottom, int right, int top)
                //                {
                //                    ROS_INFO("IN rviz::Picked ObjectOfInterestSelectionManager::pick2(Ogre::Viewport* viewport, int left, int bottom, int right, int top) %i %i %i %i", left, bottom, right, top);
                //
                //                    assert(left <= right);
                //                    assert(bottom <= top);
                //
                //                    Ogre::Camera* mCamera = viewport->getCamera();
                //                    Ogre::SceneManager* mSceneMgr = multiVizMgr->getSceneManager();
                //
                //
                //                    // Now we are at the meat of the function, and we need to perform the query itself. PlaneBoundedVolumeQueries use planes to enclose an area, then select any objects inside that area. For this example we will build an area enclosed by five planes which face inward. To create these planes out of our rectangle, we will create 4 rays, one for each corner of the rectangle. Once we have these four rays, we will grab points along the rays to create the planes: 
                //
                //                    Ogre::Ray topLeft = mCamera->getCameraToViewportRay(left, top);
                //                    Ogre::Ray topRight = mCamera->getCameraToViewportRay(right, top);
                //                    Ogre::Ray bottomLeft = mCamera->getCameraToViewportRay(left, bottom);
                //                    Ogre::Ray bottomRight = mCamera->getCameraToViewportRay(right, bottom);
                //
                //                    // Now we will create the planes. Note that we are grabbing a point 100 units along the ray. This was chosen fairly arbitrarily. We could have chosen 2 instead of 100. The only point which matters here is the front plane, which we are starting 3 units in front of the Camera.
                //
                //                    Ogre::PlaneBoundedVolume vol;
                //                    vol.planes.push_back(Ogre::Plane(topLeft.getPoint(3), topRight.getPoint(3), bottomRight.getPoint(3))); // front plane
                //                    vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), topLeft.getPoint(100), topRight.getPoint(100))); // top plane
                //                    vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), bottomLeft.getPoint(100), topLeft.getPoint(100))); // left plane
                //                    vol.planes.push_back(Ogre::Plane(bottomLeft.getOrigin(), bottomRight.getPoint(100), bottomLeft.getPoint(100))); // bottom plane
                //                    vol.planes.push_back(Ogre::Plane(topRight.getOrigin(), topRight.getPoint(100), bottomRight.getPoint(100))); // right plane
                //
                //                    // These planes have now defined an "open box" which extends to infinity in front of the camera. You can think of the rectangle we drew with the mouse as being the termination point of the box just in front of the camera. Now that we have created the planes, we need to execute the query: 
                //
                //                    Ogre::PlaneBoundedVolumeList volList;
                //                    volList.push_back(vol);
                //
                //                    Ogre::PlaneBoundedVolumeListSceneQuery* mVolQuery = mSceneMgr->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());
                //                    //mSceneMgr->destroyQuery(mVolQuery); ADD IN DESTRUCTOR
                //                    
                //                    mVolQuery->setVolumes(volList);
                //                    Ogre::SceneQueryResult result = mVolQuery->execute();
                //
                //                    ROS_INFO("Pick2 found %i results", result.movables.size());
                //                    
                //                    Ogre::SceneQueryResultMovableList::iterator iter;
                //                    for (iter = result.movables.begin(); iter != result.movables.end(); ++iter)
                //                    {
                //                        //selectObject(*iter);
                //                        std::cout << (*iter)->getName() << std::endl;
                //                    }
                //
                //                    
                //                    /////////// PICK 3
                //                    
                //                    Ogre::Ray middle = mCamera->getCameraToViewportRay((left+right)/2, (top+bottom)/2);
                //                    Ogre::RaySceneQuery*  mRaySceneQuery = mSceneMgr->createRayQuery( middle );
                //                    
                //                    Ogre::RaySceneQueryResult resultRAY = mRaySceneQuery->execute();
                //                    
                //                     ROS_INFO("Pick2 RAY found %i results", resultRAY.size());
                //                     
                //                    Ogre::RaySceneQueryResult::iterator iterRAY;
                //                    for (iterRAY = resultRAY.begin(); iterRAY != resultRAY.end(); ++iterRAY)
                //                    {
                //                        //selectObject(*iter);
                //                        std::cout << (*iterRAY).movable->getName() << std::endl;
                //                    }
                //                    
                //                    
                //                    ROS_INFO("OUT rviz::Picked ObjectOfInterestSelectionManager::pick2(Ogre::Viewport* viewport, int left, int bottom, int right, int top)");
                //                    
                //                    return rviz::Picked(0);
                //
                //                    
                //                }

                void MarkerSelectionManager::addPickTechnique(rviz::CollObjectHandle handle, const Ogre::MaterialPtr& material)
                {
                    Ogre::DataStreamPtr pixel_stream;
                    pixel_stream.bind(new Ogre::MemoryDataStream(&handle, 3));

                    Ogre::Technique* technique = 0;

                    uint32_t num_techs = material->getNumTechniques();
                    for (uint32_t i = 0; i < num_techs; ++i)
                    {
                        Ogre::Technique* tech = material->getTechnique(i);

                        if (tech->getSchemeName() == "Pick")
                        {
                            technique = tech;
                            break;
                        }
                    }

                    if (!technique)
                    {
                        technique = material->createTechnique();
                        technique->setSchemeName("Pick");
                        Ogre::Pass* pass = technique->createPass();
                        pass->setLightingEnabled(false);
                        pass->setSceneBlending(Ogre::SBT_REPLACE);

                        Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(material->getName() + "PickTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D, 0);
                        Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();
                        tex_unit->setTextureName(tex->getName());
                        tex_unit->setTextureFiltering(Ogre::TFO_NONE);
                        tex_unit->setColourOperation(Ogre::LBO_REPLACE);
                    }
                    else
                    {
                        Ogre::TextureUnitState* tex_unit = technique->getPass(0)->getTextureUnitState(0);
                        std::string tex_name = tex_unit->getTextureName();

                        Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().getByName(tex_name);
                        tex->unload();
                        tex->loadRawData(pixel_stream, 1, 1, Ogre::PF_R8G8B8);
                    }
                }

                rviz::CollObjectHandle MarkerSelectionManager::addTrackedMarker(eu::nifti::ocu::display::IUSARMarker* marker)
                {
                    assert(marker != NULL);

                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    rviz::CollObjectHandle coll = createHandle();

                    std::set<Ogre::Material*> materials;

                    const Ogre::Entity* entity = marker->getOgreIcon();

                    uint32_t num_sub_entities = entity->getNumSubEntities();
                    for (uint32_t i = 0; i < num_sub_entities; ++i)
                    {
                        Ogre::SubEntity* sub = entity->getSubEntity(i);

                        Ogre::MaterialPtr material = sub->getMaterial();

                        if (materials.insert(material.get()).second)
                        {
                            addPickTechnique(coll, material);
                        }
                    }

                    // Adds this entity to the list of maintened OGRE entities
                    trackedMarkers.insert(std::make_pair(coll, marker));

                    return coll;
                }

                void MarkerSelectionManager::removeTrackedMarker(const rviz::CollObjectHandle obj)
                {
                    assert(obj);
                    assert(selectedMarker == NULL || selectedMarker != getMarker(obj)); // Assert that we're not trying to delete the marker that is currently selected

                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    trackedMarkers.erase(obj);
                }

                /**
                 * Gets a marker from a collision handle
                 * @param obj
                 * @return 
                 */
                eu::nifti::ocu::display::IUSARMarker* MarkerSelectionManager::getMarker(rviz::CollObjectHandle obj)
                {
                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    boost::unordered_map<rviz::CollObjectHandle, eu::nifti::ocu::display::IUSARMarker*>::iterator it = trackedMarkers.find(obj);
                    if (it != trackedMarkers.end())
                    {
                        return it->second;
                    }

                    return NULL;
                }

                const eu::nifti::ocu::display::IUSARMarker* MarkerSelectionManager::getSelectedMarker()
                {
                    return selectedMarker;
                }

                void MarkerSelectionManager::deselect()
                {
                    boost::recursive_mutex::scoped_lock lock(global_mutex_);

                    if (selectedMarker == NULL)
                        return;

                    selectedMarker->onDeselect();

                    selectedMarker = NULL;
                }

                void MarkerSelectionManager::focusOnSelection()
                {
                    if (selectedMarker == NULL)
                        return;

                    // Benoit This might be an annoying behaviour because all views will change at once
                    // Maybe we could make only the "active" view change
                    multiVizMgr->lookAt(selectedMarker->getOgreIcon()->getBoundingBox().getCenter());
                }

            }
        }
    }
}