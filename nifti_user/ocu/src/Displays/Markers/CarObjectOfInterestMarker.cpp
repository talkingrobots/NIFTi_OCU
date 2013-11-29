// Benoit 2010-11-17

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ogre_tools/shape.h>

#include <Displays/OgreObjectsCreator.h>

#include "NIFTiROSUtil.h"

#include "Displays/Markers/CarObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                const std::string CarObjectOfInterestMarker::ICON_NAME = std::string("car"); // Loads the file car.dae in the icon folder
                const Ogre::ColourValue CarObjectOfInterestMarker::ICON_BOX_COLOR = Ogre::ColourValue(0, 1, 0, 0.50); // Makes the car green and half-transparent

                const float CarObjectOfInterestMarker::DEFAULT_HEIGHT_FUNC_AREAS = -0.001; // 1 mm below ground

                CarObjectOfInterestMarker::CarObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
                : ObjectOfInterestMarker(sceneMgr, frameTransformer, parent_node)
                , polygons(new std::set<Ogre::ManualObject*>())
                {
                    saveOOI(msg);
                }

                const eu_nifti_env::CarObjectOfInterestConstPtr CarObjectOfInterestMarker::getCarObjectOfInterest() const
                {
                    return eu_nifti_env::CarObjectOfInterestConstPtr(&coi);
                }

                void CarObjectOfInterestMarker::saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                {
                    ObjectOfInterestMarker::saveOOI(msg);

                    assert(msg->type == eu_nifti_env::ElementOfInterest::TYPE_CAR);

                    // Does a full copy of the data (or at least I think that it does)
                    coi = eu_nifti_env::CarObjectOfInterest(msg->car);
                    ooi = &(coi.object);
                }
                
                const std::string& CarObjectOfInterestMarker::getIconName() const
                {
                    return ICON_NAME;
                }
                    
                const Ogre::ColourValue& CarObjectOfInterestMarker::getIconBoxColor() const
                {
                    return ICON_BOX_COLOR;
                }
                
                void CarObjectOfInterestMarker::addToOgre()
                {
                    ObjectOfInterestMarker::addToOgre();
                    
                    Ogre::Vector3 scale;

                    scale.x = 4;
                    scale.y = 2;
                    scale.z = 1.5; // Half of the polygon is in the ground
                    // Problem with OGRE: if I make it higher, for example 2
                    // meters high, I don't see it anymore from the 2D view

                    ogreBox->setScale(scale);
                }

                void CarObjectOfInterestMarker::modifyInOgre(const std::string& frame_id, ros::Time stamp, const geometry_msgs::Pose& pose)
                {
                    ObjectOfInterestMarker::modifyInOgre(frame_id, stamp, pose);

                    // This will display the functional areas only if the parameter is set
                    bool showDebugDisplays = false;
                    if (NIFTiROSUtil::getParam("showDebugDisplays", showDebugDisplays) && showDebugDisplays)
                    {
                        modifyPolygons();
                    }
                }

                // to be called from addToOgre and modify
                // For now, this is inefficient because I clear all polygons and re-create them

                void CarObjectOfInterestMarker::modifyPolygons()
                {
                    //std::cout << "IN void CarObjectOfInterestMarker::modifyPolygons()" << std::endl;

                    markerNode->setVisible(false);

                    // Clears the old polygons (will regenerate every thing from the car of interest)
                    clearPolygons();

                    int numPolygon = 1;
                    std::vector<eu_nifti_env::FunctionalArea>::const_iterator it_begin = coi.functionalAreas.begin();
                    std::vector<eu_nifti_env::FunctionalArea>::const_iterator it_end = coi.functionalAreas.end();
                    for (; it_begin != it_end; it_begin++)
                    {
                        const eu_nifti_env::FunctionalArea& funcArea = *it_begin;

                        std::stringstream polygonName;
                        polygonName << "Element UUID #" << ooi->element.uuid << " / " << funcArea.function << " / Polygon #" << numPolygon++;

                        //std::cout << "Creating func area: " << polygonName.str() << std::endl;

                        // Attaches the polygon to the marker node rather than the scene node like before.
                        try
                        {
                            const Ogre::ColourValue& color = eu::nifti::ocu::display::OgreObjectsCreator::getColor(ooi->element.uuid % 125);

                            Ogre::ManualObject* polygon = sceneMgr->createManualObject(polygonName.str());
                            OgreObjectsCreator::createOgreGroundPolygon(funcArea.area, polygon, color, DEFAULT_HEIGHT_FUNC_AREAS, markerNode);
                            //OgreObjectsCreator::createOgreGroundPolygon(funcArea.area, polygon, color, DEFAULT_HEIGHT_FUNC_AREAS);

                            polygons->insert(polygon);
                            markerNode->attachObject(polygon);
                            //sceneMgr->getRootSceneNode()->attachObject(polygon);
                        } catch (Ogre::Exception& ex)
                        {
                            std::cerr << "An Ogre exception has occured while : " << ex.getFullDescription().c_str() << std::endl;
                        }


                    }

                    markerNode->setVisible(true);

                    //std::cout << "OUT void CarObjectOfInterestMarker::modifyPolygons()" << std::endl;
                }

                void CarObjectOfInterestMarker::clearPolygons()
                {
                    for (std::set<Ogre::ManualObject*>::iterator it = polygons->begin(); it != polygons->end(); it++)
                    {
                        sceneMgr->destroyManualObject(*it); // This handles de-allocation of memory
                    }
                    polygons->clear();
                }


            }
        }
    }
}

