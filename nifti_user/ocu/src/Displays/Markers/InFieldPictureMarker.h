// Benoit 2013-01-18

#ifndef EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_MARKER_H

#include <string>

#include <OGRE/OgreMaterial.h>

//#include <ros/time.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

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

                class InFieldPictureMarker : public IUSARMarker
                {
                public:
                    InFieldPictureMarker(Ogre::SceneManager* sceneMgr, Ogre::SceneNode* parent_node, const EXIFReader_msgs::AnnotatedPicture* picture);
                    ~InFieldPictureMarker();

                    const std::string& getIdentifier() const;
                    
                    const EXIFReader_msgs::AnnotatedPicture* getPicture() const;

                    const Ogre::Entity* getOgreIcon() const;

                    void addToOgre();

                    // This is used for the selection
                    rviz::CollObjectHandle getCollisionObjectHandle() const;
                    void setCollisionObjectHandle(rviz::CollObjectHandle handle);

                    void onSelect();
                    void onDeselect();

                    // From IUSARMarker
                    IUSARMarker::Type getMarkerType() const;

                protected:

                    void createOgreObject();
                    void createLabel();

                    const EXIFReader_msgs::AnnotatedPicture* picture;

                    // Members related to OGRE
                    Ogre::SceneManager* sceneMgr;
                    ogre_tools::MovableText* ogreLabel;
                    Ogre::Entity* ogreObject;
                    Ogre::SceneNode* ogreNode;
                    //Ogre::SceneNode* markerNode;
                    //Ogre::SceneNode* iconNode;
                    Ogre::SceneNode* labelNode;
                    Ogre::MaterialPtr material_;
                    std::string material_name_;

                    // This is a unique identifier used by the selection manager to keep track of this object
                    rviz::CollObjectHandle collisionObjectHandle;

                };


            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_IN_FIELD_PICTURE_MARKER_H


