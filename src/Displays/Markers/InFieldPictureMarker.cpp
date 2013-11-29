// Benoit 2013-01-18

#include <boost/date_time/posix_time/posix_time.hpp>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ogre_tools/movable_text.h>

#include "NIFTiConstants.h"

#include "Displays/Markers/mesh_loader.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/Markers/InFieldPictureMarker.h"

using namespace std;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                const float LABEL_SIZE = 0.4; // Capital letters are 0.40 m high
                const float LABEL_HEIGHT = 0.5; // The label floats 0.5 meter above the object

                const float ICON_SCALING = 0.025; // With respect to the model (.dae file)
                const Ogre::ColourValue ICON_COLOR = Ogre::ColourValue(0.3, 1, 1, 0.1); // Light blue and semi-transparent
                const string MODEL_FILENAME = "camera_light_centered.dae";

                InFieldPictureMarker::InFieldPictureMarker(Ogre::SceneManager* sceneMgr, Ogre::SceneNode* parent_node, const EXIFReader_msgs::AnnotatedPicture* picture)
                : picture(picture)
                , sceneMgr(sceneMgr)
                , ogreObject(NULL)
                , collisionObjectHandle(0)
                {
                    assert(parent_node != NULL);

                    ogreNode = parent_node->createChildSceneNode();
                    //markerNode = parent_node->createChildSceneNode();
                    labelNode = parent_node->createChildSceneNode(); // I don't make it inherit the marker node so that I can use the marker's box's tight bounding box for selection
                    //iconNode = markerNode->createChildSceneNode();
                }

                InFieldPictureMarker::~InFieldPictureMarker()
                {
                    //std::cout << "IN InFieldPictureMarker::~InFieldPictureMarker(): " << picture->filename << std::endl;

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

                    sceneMgr->destroySceneNode(ogreNode);
                    //sceneMgr->destroySceneNode(markerNode);
                    sceneMgr->destroySceneNode(labelNode);
                    //sceneMgr->destroySceneNode(iconNode);

                    //std::cout << "OUT InFieldPictureMarker::~InFieldPictureMarker(): " << picture->filename << std::endl;
                }
                
                const std::string& InFieldPictureMarker::getIdentifier() const
                {
                    return picture->filename;
                }

                const Ogre::Entity* InFieldPictureMarker::getOgreIcon() const
                {
                    return ogreObject;
                }

                const AnnotatedPicture* InFieldPictureMarker::getPicture() const
                {
                    return picture;
                }

                void InFieldPictureMarker::addToOgre()
                {
                    assert(!ogreObject);

                    ogreNode->setVisible(false);

                    createOgreObject();
                    createLabel();

                    ogreNode->setPosition(picture->pose.position.x, picture->pose.position.y, picture->pose.position.z);

                    // If it's all zeros, then there is no orientation provided. Just point anywhere
                    if (picture->pose.orientation.w == 0 && picture->pose.orientation.x == 0 && picture->pose.orientation.y == 0 && picture->pose.orientation.z == 0)
                    {
                        // Do nothing
                    }
                    else
                    {
                        ogreNode->setOrientation(picture->pose.orientation.w, picture->pose.orientation.x, picture->pose.orientation.y, picture->pose.orientation.z);
                    }

                    // Places the label node a bit higher
                    labelNode->setPosition(picture->pose.position.x, picture->pose.position.y, picture->pose.position.z + LABEL_HEIGHT);

                    ogreNode->setVisible(true);
                }

                void InFieldPictureMarker::createOgreObject()
                {
                    //std::cout << "IN void NIFTiObjectMarker::createObject(const MarkerConstPtr & new_message)" << std::endl;

                    std::stringstream ssPath;
                    ssPath << "file://" << NIFTiConstants::ROS_PACKAGE_PATH << "/media/OGRE/meshes/" << MODEL_FILENAME;
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
                    ss << "Mesh Resource Marker - In-Field Pictures" << count++;
                    ogreObject = sceneMgr->createEntity(ss.str(), mesh_resource_path);
                    ogreNode->attachObject(ogreObject);
                    ogreNode->setScale(ICON_SCALING, ICON_SCALING, ICON_SCALING); // Shrinks the objects by the factor defined above

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

                    // Makes the icons the appropriate color
                    material_->getTechnique(0)->setAmbient(ICON_COLOR / 2);
                    material_->getTechnique(0)->setDiffuse(ICON_COLOR);

                }

                void InFieldPictureMarker::createLabel()
                {
                    string labelText;

                    if (picture->dateTimeFormatted.length() == 19)
                    {
                        labelText = picture->dateTimeFormatted.substr(11);
                    }
                    else if (!picture->dateTime.isZero())
                    {
                        // Formats the date and time and leaves out the extras
                        //labelText = boost::posix_time::to_iso_extended_string(picture->dateTime.toBoost()).substr(0, 10) + " " + boost::posix_time::to_iso_extended_string(picture->dateTime.toBoost()).substr(11, 8);
                        labelText = boost::posix_time::to_iso_extended_string(picture->dateTime.toBoost()).substr(11, 8);
                    }
                    else
                    {
                        labelText = "No Time";
                    }

                    ogreLabel = new ogre_tools::MovableText(labelText);
                    ogreLabel->setColor(Ogre::ColourValue::Red);
                    ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
                    ogreLabel->setCharacterHeight(LABEL_SIZE);
                    labelNode->attachObject(ogreLabel);
                }

                rviz::CollObjectHandle InFieldPictureMarker::getCollisionObjectHandle() const
                {
                    return collisionObjectHandle;
                }

                void InFieldPictureMarker::setCollisionObjectHandle(rviz::CollObjectHandle handle)
                {
                    collisionObjectHandle = handle;
                }

                void InFieldPictureMarker::onSelect()
                {
                    ogreNode->showBoundingBox(true);
                }

                void InFieldPictureMarker::onDeselect()
                {
                    ogreNode->showBoundingBox(false);
                }
                
                IUSARMarker::Type InFieldPictureMarker::getMarkerType() const
                {
                    return IUSARMarker::ANNOTATED_PICTURE;
                }

            }
        }
    }
}

