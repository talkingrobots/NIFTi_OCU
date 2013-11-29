// Benoit 2010-11-17 Based on the RVIZ MeshResourceMarker class

/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ogre_tools/movable_text.h>
#include <ogre_tools/shape.h>

#include <eu_nifti_env/ObjectOfInterest.h>

#include "FloatValidator.h"
#include "NIFTiConstants.h"

#include "Displays/Markers/mesh_loader.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/Markers/ObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                const float ObjectOfInterestMarker::LABEL_SIZE = 0.6; // Capital letters are 0.60 m high
                const float ObjectOfInterestMarker::LABEL_HEIGHT = 0.8; // The label floats 0.5 meter above the object

                const float ObjectOfInterestMarker::ICON_SCALING = 0.333; // The icons are 0.2 times their real size
                const float ObjectOfInterestMarker::ICON_FLOATING_HEIGHT = 0.1; // The icons float a 75 centimeters above the ground
                const Ogre::ColourValue ObjectOfInterestMarker::ICON_COLOR = Ogre::ColourValue(1, 1, 1, 1); // White and opaque

                ObjectOfInterestMarker::ObjectOfInterestMarker(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
                : sceneMgr(sceneMgr)
                , frameTransformer(frameTransformer)
                , ogreLabel(0)
                , ogreObject(0)
                , ogreBox(0)
                , collisionObjectHandle(0)
                {
                    assert(parent_node != NULL);

                    markerNode = parent_node->createChildSceneNode();
                    labelNode = parent_node->createChildSceneNode(); // I don't make it inherit the marker node so that I can use the marker's box's tight bounding box for selection
                    iconNode = markerNode->createChildSceneNode();
                }

                ObjectOfInterestMarker::~ObjectOfInterestMarker()
                {
                    if (ogreObject)
                    {
                        // Todo Benoit Check if this is necessary, since we destroy the node containing the object
                        sceneMgr->destroyEntity(ogreObject);
                        labelNode->detachAllObjects();

                        for (size_t i = 0; i < material_->getNumTechniques(); ++i)
                        {
                            Ogre::Technique* t = material_->getTechnique(i);
                            // hack hack hack, really need to do a shader-based way of picking, rather than
                            // creating a texture for each object
                            if (t->getSchemeName() == "Pick")
                            {
                                Ogre::TextureManager::getSingleton().remove(t->getPass(0)->getTextureUnitState(0)->getTextureName());
                            }
                        }

                        material_->unload();
                        Ogre::MaterialManager::getSingleton().remove(material_->getName());
                    }

                    sceneMgr->destroySceneNode(markerNode);
                }

                const std::string& ObjectOfInterestMarker::getIdentifier() const
                {
                    return ooi->element.name; // This should be unique, although it is probably not checked
                }

                const Ogre::Entity* ObjectOfInterestMarker::getOgreIcon() const
                {
                    // Makes the box around the icon selectable, rather than the icon
                    return ogreBox->getEntity();

                    //return ogreObject;
                }

                //                const Ogre::Entity* ObjectOfInterestMarker::getOgreBox() const
                //                {
                //                    return ogreBox->getEntity();
                //                }

                const eu_nifti_env::ObjectOfInterest* ObjectOfInterestMarker::getObjectOfInterest() const
                {
                    return ooi;
                }

                const eu_nifti_cast::WorkingMemoryPointer& ObjectOfInterestMarker::getCASTWorkingMemoryPointer() const
                {
                    return castWorkingMemoryPointer;
                }

                void ObjectOfInterestMarker::saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                {
                    castWorkingMemoryPointer = msg->castWorkingMemoryPointer;

                    // This was for a test. Good idea to keep it.
                    //eu_nifti_cast::WorkingMemoryAddress a;
                    //a.id = "42342";
                    //a.subarchitecture = "sa foo";
                    //std::string type = "my type";
                    //castWorkingMemoryPointer.address = a;
                    //castWorkingMemoryPointer.type = type;
                }

                void ObjectOfInterestMarker::addToOgre()
                {
                    assert(!ogreObject);

                    markerNode->setVisible(false);

                    createIcon();
                    createLabel();

                    ogreBox = new ogre_tools::Shape(ogre_tools::Shape::Cube, sceneMgr, markerNode);

                    markerNode->setVisible(true);
                }

                void ObjectOfInterestMarker::modifyInOgre(const std::string& frame_id, ros::Time stamp, const geometry_msgs::Pose& pose)
                {
                    //markerNode->setVisible(false);

                    ////////////////////////////////
                    //
                    // 1) Adjusts the color
                    //
                    ////////////////////////////////

                    // If objects come and they are not confirmed by the user, then they are shown differently
                    if (ooi->element.status == eu_nifti_env::ElementOfInterest::STATUS_UNCONFIRMED)
                    {
                        Ogre::ColourValue c = getIconBoxColor();
                        c.r = 0.1;
                        c.g = 0.1;
                        c.b = 0.1;
                        c.a = 1;
                        ogreBox->setColor(c);
                    }
                    else
                    {
                        ogreBox->setColor(getIconBoxColor());
                    }


                    ////////////////////////////////
                    //
                    // 2) Adjusts the position
                    //
                    ////////////////////////////////

                    Ogre::Vector3 ogrePosition;
                    Ogre::Quaternion ogreOrientation;

//                    ////////////////////////////////
//                    // WARNING!!! This is a hack made on 2011-11-30 because Shanker's component sends me cars at 3 meters in the air
//
//                    if (pose.position.z != 0)
//                        ROS_WARN("Received an EOI at %f meters above the ground. The GUI will artificially put it down to the ground.", pose.position.z);
//
//                    geometry_msgs::Pose poseCorrected = pose;
//                    poseCorrected.position.z = 0;
//                    ////////////////////////////////

                    // Tries to perform the transform to the fixed frame
                    if (!frameTransformer->transform(frame_id, stamp, pose, ogrePosition, ogreOrientation)
                            || !rviz::FloatValidator::validateFloats(pose))
                    {
                        std::string error;
                        frameTransformer->transformHasProblems(frame_id, stamp, error);
                        throw error;
                    }

                    // The position given refers to the position at the ground level

                    ogrePosition.z += ICON_FLOATING_HEIGHT; // Makes the icons float a few centimeters above the ground

                    markerNode->setPosition(ogrePosition);
                    markerNode->setOrientation(ogreOrientation);

                    ogrePosition.z += LABEL_HEIGHT;

                    labelNode->setPosition(ogrePosition);
                    labelNode->setOrientation(ogreOrientation);


                    //markerNode->setVisible(true);

                }

                void ObjectOfInterestMarker::createLabel()
                {
                    //std::cout << "IN void NIFTiObjectMarker::addLabel(const MarkerConstPtr & new_message)" << std::endl;
                    std::ostringstream oss;
                    oss << getIconName() << ooi->element.uuid;
                    //oss << msg->ns << msg->id;
                    ogreLabel = new ogre_tools::MovableText(oss.str());
                    ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
                    ogreLabel->setCharacterHeight(LABEL_SIZE);
                    labelNode->attachObject(ogreLabel);
                }

                void ObjectOfInterestMarker::createIcon()
                {
                    //std::cout << "IN void NIFTiObjectMarker::createObject(const MarkerConstPtr & new_message)" << std::endl;

                    std::stringstream ssPath;
                    ssPath << "file://" << NIFTiConstants::ROS_PACKAGE_PATH << "/media/OGRE/meshes/" << getIconName() << ".dae";
                    std::string mesh_resource_path = ssPath.str();

                    // Benoit todo: Load the meshes in advance, if possible
                    if (rviz::loadMeshFromResource(mesh_resource_path).isNull())
                    {
                        std::cerr << "Could not load mesh resource" << mesh_resource_path << "for marker" << std::endl;
                        throw "Could not load mesh resource";
                    }

                    // Creates an OGRE object and attaches it to the scene node
                    static uint32_t count = 0;
                    std::stringstream ss;
                    ss << "Mesh Resource Marker" << count++;
                    ogreObject = sceneMgr->createEntity(ss.str(), mesh_resource_path);
                    iconNode->attachObject(ogreObject);
                    iconNode->setScale(ICON_SCALING, ICON_SCALING, ICON_SCALING); // Shrinks the objects by 1/3 to look like an icon

                    // Sets-up the material used for the object
                    ss << "Material";
                    material_name_ = ss.str();

                    //std::cout << "material_name_" << material_name_ << std::endl;

                    material_ = Ogre::MaterialManager::getSingleton().create(material_name_, ROS_PACKAGE_NAME);
                    material_->setReceiveShadows(false);
                    material_->getTechnique(0)->setLightingEnabled(true);
                    material_->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);

                    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
                    material_->getTechnique(0)->setDepthWriteEnabled(true);

                    ogreObject->setMaterialName(material_name_);

                    //iconNode->showBoundingBox(true);

                    // Makes the icons the appropriate color
                    material_->getTechnique(0)->setAmbient(ICON_COLOR / 2);
                    material_->getTechnique(0)->setDiffuse(ICON_COLOR);

                }

                rviz::CollObjectHandle ObjectOfInterestMarker::getCollisionObjectHandle() const
                {
                    return collisionObjectHandle;
                }

                void ObjectOfInterestMarker::setCollisionObjectHandle(rviz::CollObjectHandle handle)
                {
                    collisionObjectHandle = handle;
                }

                void ObjectOfInterestMarker::onSelect()
                {
                    //iconNode->showBoundingBox(true);
                    markerNode->showBoundingBox(true);
                    //labelNode->showBoundingBox(true);
                }

                void ObjectOfInterestMarker::onDeselect()
                {
                    //iconNode->showBoundingBox(false);
                    markerNode->showBoundingBox(false);
                    //labelNode->showBoundingBox(false);
                }

                IUSARMarker::Type ObjectOfInterestMarker::getMarkerType() const
                {
                    return IUSARMarker::ELEMENT_OF_INTEREST;
                }

            }
        }
    }
}

