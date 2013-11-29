// Benoit 2012-06-22

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <geometry_msgs/Point32.h>

#include "Displays/OgreObjectsCreator.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "NIFTiViewsUtil.h"

#include "Displays/DistanceDisplay.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                DistanceDisplay::DistanceDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
                : rviz::Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
                , sceneNode(sceneMgr->getRootSceneNode()->createChildSceneNode())
                {
                    createCircles();

                    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ""); // Always leaves the status at OK, because this display reads nothing else than the TFs
                }

                DistanceDisplay::~DistanceDisplay()
                {
                    sceneMgr->destroySceneNode(sceneNode);
                }

                void DistanceDisplay::onEnable()
                {
                    sceneNode->setVisible(true);
                }

                void DistanceDisplay::onDisable()
                {
                    sceneNode->setVisible(false);
                }

                void DistanceDisplay::fixedFrameChanged()
                {
                }

                void DistanceDisplay::update(float wall_dt, float ros_dt)
                {
                    //std::cout << "void DistanceDisplay::update(float wall_dt, float ros_dt) " << this << std::endl;

                    updatePositionOfSceneNode();
                }

                void DistanceDisplay::updatePositionOfSceneNode()
                {
                    //std::cout << "IN void DistanceDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << this << " at time " << ros::Time::now() << std::endl;

                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;
                    if (!frameTransformer->getTransform("base_link", ros::Time(0), position, orientation))
                    {
                        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", "base_link", fixed_frame_.c_str());
                    }

                    sceneNode->setPosition(position);
                    sceneNode->setOrientation(orientation);

                    //std::cout << "OUT void DistanceDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)" << std::endl;
                }

                void DistanceDisplay::createCircles()
                {

                    // Todo: put these values in the class as constants
                    std::vector<double > distances;
                    distances.push_back(1);
                    distances.push_back(2);
                    distances.push_back(5);
                    
                    std::vector<Ogre::ColourValue > colors;
                    colors.push_back(Ogre::ColourValue(0.75, 0, 0, 0.75)); // Red
                    colors.push_back(Ogre::ColourValue(0.75, 0.75, 0, 0.75)); // Yellow
                    colors.push_back(Ogre::ColourValue(0, 0.75, 0, 0.75)); // Green

                    for (u_int polygonNumber = 0; polygonNumber < distances.size(); polygonNumber++)
                    {       
                        
                        std::stringstream circleName;
                        circleName << "Safety Distance " << polygonNumber << " meters";
                        
                        Ogre::ManualObject* circle = sceneMgr->createManualObject(circleName.str());
                        OgreObjectsCreator::makeCircle(circle, distances.at(polygonNumber), colors.at(polygonNumber));                        
                        sceneNode->attachObject(circle);
                        
                        circle->setVisibilityFlags(NIFTiViewsUtil::getVisiblityFlag(this->getName())); // NIFTi: Allows hiding in Map View
                        
                        
//                        // This would add labels to the circle, to remind the user of the distance
//                        
//                        std::stringstream labelText;
//                        labelText << distances.at(polygonNumber);
//                        ogre_tools::MovableText* ogreLabel = new ogre_tools::MovableText(labelText.str());
//                        ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
//                        ogreLabel->setColor(colors.at(polygonNumber));
//                        ogreLabel->setCharacterHeight(0.25);
//                        Ogre::SceneNode* labelNode = sceneNode->createChildSceneNode();
//                        labelNode->attachObject(ogreLabel);
//                        labelNode->setPosition(0, 0.1, -1);
//                        
//                        ogreLabel = new ogre_tools::MovableText(labelText.str());
//                        ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
//                        ogreLabel->setColor(colors.at(polygonNumber));
//                        ogreLabel->setCharacterHeight(0.25);
//                        labelNode = sceneNode->createChildSceneNode();
//                        labelNode->attachObject(ogreLabel);
//                        labelNode->setPosition(-1, 0.1, 0);
//                        
//                        ogreLabel = new ogre_tools::MovableText(labelText.str());
//                        ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
//                        ogreLabel->setColor(colors.at(polygonNumber));
//                        ogreLabel->setCharacterHeight(0.25);
//                        labelNode = sceneNode->createChildSceneNode();
//                        labelNode->attachObject(ogreLabel);
//                        labelNode->setPosition(1, 0.1, 0);
//                        
//                        ogreLabel = new ogre_tools::MovableText(labelText.str());
//                        ogreLabel->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
//                        ogreLabel->setColor(colors.at(polygonNumber));
//                        ogreLabel->setCharacterHeight(0.25);
//                        labelNode = sceneNode->createChildSceneNode();
//                        labelNode->attachObject(ogreLabel);
//                        labelNode->setPosition(0, 0.1, 1);
                        

                    } // end for loop

                }

            }
        }
    }
}

