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

#ifndef EU_NIFTI_OCU_DISPLAY_OOI_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_OOI_MARKER_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <OGRE/OgreMaterial.h>

#include <ros/time.h>

#include <eu_nifti_cast/WorkingMemoryPointer.h>

#include <eu_nifti_env/ObjectOfInterest.h>

#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

#include "IUSARMarker.h"

#include "Selection/Common.h"

namespace Ogre
{
    class SceneManager;
    class SceneNode;
    class Vector3;
    class Quaternion;
    class Entity;
}

namespace ogre_tools
{
    class MovableText;
    class Shape;
}

namespace rviz
{
    class FrameTransformer;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                class ObjectOfInterestMarker : public IUSARMarker
                {

                public:
                    ObjectOfInterestMarker(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node);
                    ~ObjectOfInterestMarker();

                    // The subclasses must implement this method
                    virtual void saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);

                    const eu_nifti_env::ObjectOfInterest* getObjectOfInterest() const;

                    const std::string& getIdentifier() const;
                    
                    const Ogre::Entity* getOgreIcon() const;
                    //const Ogre::Entity* getOgreBox() const;
                    
                    const eu_nifti_cast::WorkingMemoryPointer& getCASTWorkingMemoryPointer() const;
                    
                    virtual void addToOgre();
                    virtual void modifyInOgre(const std::string& frame_id, ros::Time stamp, const geometry_msgs::Pose& pose);
                    
                    // This is used for the selection
                    rviz::CollObjectHandle getCollisionObjectHandle() const;
                    void setCollisionObjectHandle(rviz::CollObjectHandle handle);
                    
                    virtual void onSelect();
                    virtual void onDeselect();
                    
                    // From IUSARMarker
                    IUSARMarker::Type getMarkerType() const;

                protected:

                    void transform(const std::string& frame_id, ros::Time stamp, const geometry_msgs::Pose& pose, Ogre::Vector3& pos, Ogre::Quaternion& orient, Ogre::Vector3& scale);

                    void createLabel();
                    void createIcon();
                    
                    virtual const std::string& getIconName() const = 0;
                    virtual const Ogre::ColourValue& getIconBoxColor() const = 0;
                    
                    eu_nifti_env::ObjectOfInterest* ooi;
                    
                    // This is just there to help CAST when I send it selection messages
                    eu_nifti_cast::WorkingMemoryPointer castWorkingMemoryPointer;
                    
                    // Members related to OGRE management
                    Ogre::SceneManager* sceneMgr;
                    rviz::FrameTransformer* frameTransformer;

                    // Members related to OGRE
                    ogre_tools::MovableText* ogreLabel;
                    Ogre::Entity* ogreObject;
                    ogre_tools::Shape* ogreBox;
                    Ogre::SceneNode* markerNode;
                    Ogre::SceneNode* iconNode;
                    Ogre::SceneNode* labelNode;
                    Ogre::MaterialPtr material_;
                    std::string material_name_;
                    
                    // This is a unique identifier used by the selection manager to keep track of this object
                    rviz::CollObjectHandle collisionObjectHandle;

                    static const float LABEL_SIZE;
                    static const float LABEL_HEIGHT;

                    static const float ICON_SCALING;
                    static const float ICON_FLOATING_HEIGHT;
                    static const Ogre::ColourValue ICON_COLOR;
                };

                typedef boost::shared_ptr<ObjectOfInterestMarker> ObjectOfInterestMarkerPtr;

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_OOI_MARKER_H


