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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "mesh_resource_marker.h"

#include "Displays/MarkerDisplay.h"

#include "Displays/Markers/mesh_loader.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz
{

MeshResourceMarker::MeshResourceMarker(MarkerDisplay* owner, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
: MarkerBase(owner, sceneMgr, frameTransformer, parent_node)
, entity_(0)
{
  if (parent_node)
  {
    scene_node_ = parent_node->createChildSceneNode();
  }
  else
  {
    scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();
  }
}

MeshResourceMarker::~MeshResourceMarker()
{
  sceneMgr->destroySceneNode(scene_node_->getName());

  if (entity_)
  {
    sceneMgr->destroyEntity( entity_ );
  }

  if (!material_.isNull())
  {
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
}

void MeshResourceMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::MESH_RESOURCE);

  scene_node_->setVisible(false);

  if (!entity_ || old_message->mesh_resource != new_message->mesh_resource)
  {
    if (entity_)
    {
      sceneMgr->destroyEntity(entity_);
      entity_ = 0;
    }

    if (new_message->mesh_resource.empty())
    {
      return;
    }

    if (loadMeshFromResource(new_message->mesh_resource).isNull())
    {
      std::stringstream ss;
      ss << "Mesh resource marker [" << getStringID() << "] could not load [" << new_message->mesh_resource << "]";
      owner_->setMarkerStatus(getID(), eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
      ROS_DEBUG("%s", ss.str().c_str());
      return;
    }

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Mesh Resource Marker" << count++;
    entity_ = sceneMgr->createEntity(ss.str(), new_message->mesh_resource);
    scene_node_->attachObject(entity_);

    if (material_.isNull())
    {
      ss << "Material";
      material_name_ = ss.str();
      material_ = Ogre::MaterialManager::getSingleton().create( material_name_, ROS_PACKAGE_NAME );
      material_->setReceiveShadows(false);
      material_->getTechnique(0)->setLightingEnabled(true);
      material_->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
    }

    original_material_names_.clear();
    for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
    {
      std::string name = entity_->getSubEntity(i)->getMaterialName();
      ss << name;
      if (!Ogre::MaterialManager::getSingleton().resourceExists(ss.str()))
      {
        Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(name);
        mat->clone(ss.str());
      }

      entity_->getSubEntity(i)->setMaterialName(ss.str());
      original_material_names_.push_back(ss.str());
    }

  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  scene_node_->setVisible(true);
  scene_node_->setPosition(pos);
  scene_node_->setOrientation(orient);
  scene_node_->setScale(scale);

  if (new_message->mesh_use_embedded_materials)
  {
    for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
    {
      entity_->getSubEntity(i)->setMaterialName(original_material_names_[i]);
    }
  }
  else
  {
    entity_->setMaterialName(material_name_);
  }

  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;
  material_->getTechnique(0)->setAmbient( r*0.5, g*0.5, b*0.5 );
  material_->getTechnique(0)->setDiffuse( r, g, b, a );

  if ( a < 0.9998 )
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }
}

}

