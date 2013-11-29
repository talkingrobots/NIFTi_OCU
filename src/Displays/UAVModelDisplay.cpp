// Benoit 2012-06-22

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <geometry_msgs/Point32.h>

#include "Displays/OgreObjectsCreator.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/UAVModelDisplay.h"

using namespace Ogre;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                const float RADIUS_HULL = 0.1;
                const float RADIUS_PROTECTION_RING = 0.425;

                UAVModelDisplay::UAVModelDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
                : rviz::Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
                , sceneNode(sceneMgr->getRootSceneNode()->createChildSceneNode())
                {
                    createOgreModel();

                    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ""); // Always leaves the status at OK, because this display reads nothing else than the TFs
                }

                UAVModelDisplay::~UAVModelDisplay()
                {
                    sceneMgr->destroySceneNode(sceneNode);
                }

                void UAVModelDisplay::onEnable()
                {
                    sceneNode->setVisible(true);
                }

                void UAVModelDisplay::onDisable()
                {
                    sceneNode->setVisible(false);
                }

                void UAVModelDisplay::fixedFrameChanged()
                {
                }

                void UAVModelDisplay::update(float wall_dt, float ros_dt)
                {
                    //std::cout << "void UAVModelDisplay::update(float wall_dt, float ros_dt) " << this << std::endl;

                    updatePositionOfSceneNode();
                }

                void UAVModelDisplay::updatePositionOfSceneNode()
                {
                    //std::cout << "IN void UAVModelDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << this << " at time " << ros::Time::now() << std::endl;

                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;
                    if (!frameTransformer->getTransform("uav_base_link", ros::Time(0), position, orientation))
                    {
                        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", "uav_base_link", fixed_frame_.c_str());
                    }

                    sceneNode->setPosition(position);
                    sceneNode->setOrientation(orientation);

                    //std::cout << "OUT void UAVModelDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)" << std::endl;
                }

                void UAVModelDisplay::createOgreModel()
                {

                    const Ogre::Vector3 ORIGIN(0, 0, 0);
                    const Ogre::ColourValue RED(1, 0, 0, 1);
                    const Ogre::ColourValue BLACK(0, 0, 0, 1);

                    {
                        Ogre::ManualObject *hull = sceneMgr->createManualObject("Hull");
                        OgreObjectsCreator::makeSphere(hull, RED, RADIUS_HULL);
                        sceneNode->attachObject(hull);
                    }

                    {
                        Ogre::ManualObject* ring = sceneMgr->createManualObject("Protection Ring");
                        OgreObjectsCreator::makeCircle(ring, RADIUS_PROTECTION_RING, RED); // Radius of the protection ring
                        sceneNode->attachObject(ring);
                    }

                    Ogre::ManualObject* line;

                    {
                        line = sceneMgr->createManualObject("Line front");
                        OgreObjectsCreator::makeLine(line, ORIGIN, Ogre::Vector3(RADIUS_PROTECTION_RING, 0, 0), RED);
                        sceneNode->attachObject(line);
                    }

                    {
                        line = sceneMgr->createManualObject("Line right");
                        OgreObjectsCreator::makeLine(line, ORIGIN, Ogre::Vector3(0, RADIUS_PROTECTION_RING, 0), BLACK);
                        sceneNode->attachObject(line);
                    }

                    {
                        line = sceneMgr->createManualObject("Line left");
                        OgreObjectsCreator::makeLine(line, ORIGIN, Ogre::Vector3(0, -RADIUS_PROTECTION_RING, 0), BLACK);
                        sceneNode->attachObject(line);
                    }

                    {
                        line = sceneMgr->createManualObject("Line back");
                        OgreObjectsCreator::makeLine(line, ORIGIN, Ogre::Vector3(-RADIUS_PROTECTION_RING, 0, 0), BLACK);
                        sceneNode->attachObject(line);
                    }


                }

            }
        }
    }
}

