// Benoit 2011-03-23
// Inspired from RVIZ polygon_display

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/Point32.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "FloatValidator.h"

#include "Displays/NIFTiPoseDisplay.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                // Makes the pose semi-transparent red
                const Ogre::ColourValue NIFTiPoseDisplay::COLOR = Ogre::ColourValue(1.0, 0.0, 0.0, 0.5);

                const char* NIFTiPoseDisplay::ROS_TOPIC = "/move_base/local_costmap/robot_footprint";

                const float NIFTiPoseDisplay::DEFAULT_HEIGHT = 0.03; // 3 cm above ground, to clear the topological segmentation markers that are 2.5 cm high

                NIFTiPoseDisplay::NIFTiPoseDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
                : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
                , msgReceivedSinceLastClear(0)
                , sceneNode(sceneMgr->getRootSceneNode()->createChildSceneNode())
                , tf_filter_(*frameTransformer->getTFClient(), "", 10, update_nh_)
                {
                    posePolygon = sceneMgr->createManualObject("posePolygon");
                    posePolygon->setDynamic(true);
                    sceneNode->attachObject(posePolygon);

                    tf_filter_.connectInput(sub_);
                    tf_filter_.registerCallback(boost::bind(&NIFTiPoseDisplay::processMessage, this, _1));
                    frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);
                }

                NIFTiPoseDisplay::~NIFTiPoseDisplay()
                {
                    unsubscribe();
                    clear();

                    sceneMgr->destroyManualObject(posePolygon);
                    sceneMgr->destroySceneNode(sceneNode);
                }

                void NIFTiPoseDisplay::clear()
                {
                    posePolygon->clear();

                    msgReceivedSinceLastClear = 0;
                    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
                }

                void NIFTiPoseDisplay::subscribe()
                {
                    if (!isEnabled())
                    {
                        return;
                    }

                    sub_.subscribe(update_nh_, ROS_TOPIC, 10);
                }

                void NIFTiPoseDisplay::unsubscribe()
                {
                    sub_.unsubscribe();
                }

                void NIFTiPoseDisplay::onEnable()
                {
                    sceneNode->setVisible(true);
                    subscribe();
                }

                void NIFTiPoseDisplay::onDisable()
                {
                    unsubscribe();
                    clear();
                    sceneNode->setVisible(false);
                }

                void NIFTiPoseDisplay::fixedFrameChanged()
                {
                    clear();

                    tf_filter_.setTargetFrame(fixed_frame_);
                }

                void NIFTiPoseDisplay::update(float wall_dt, float ros_dt)
                {
                    //std::cout << "void NIFTiPoseDisplay::update(float wall_dt, float ros_dt) " << this << std::endl;
                }

                void NIFTiPoseDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
                {
                    //std::cout << "IN void NIFTiPoseDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << ros::Time::now() << std::endl;

                    assert(msg != NULL);

                    if (!rviz::FloatValidator::validateFloats(msg->polygon.points))
                    {
                        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Message contained invalid floating point values (nans or infs)");
                        return;
                    }


                    std::stringstream ss;
                    ss << ++msgReceivedSinceLastClear << " messages received";
                    setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());


                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;
                    if (!frameTransformer->getTransform(msg->header, position, orientation))
                    {
                        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str());
                    }

                    // Moves the scene node, which in turn moves the polygon
                    sceneNode->setPosition(position);
                    sceneNode->setOrientation(orientation);

                    // Recreates the polygon from scratch
                    posePolygon->clear();
                    uint32_t num_points = msg->polygon.points.size();

                    assert(num_points != 0);

                    posePolygon->estimateVertexCount(num_points);
                    posePolygon->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);

                    // Adds the point to the polygon in the reverse order, because it is not published properly
                    for (int i = num_points - 1; i >= 0; i--)
                    {
                        Ogre::Vector3 pos(msg->polygon.points[i].x, msg->polygon.points[i].y, DEFAULT_HEIGHT);
                        posePolygon->position(pos);
                        posePolygon->colour(COLOR);
                    }

                    posePolygon->end();


                    //std::cout << "OUT void NIFTiPoseDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)" << std::endl;
                }

                void NIFTiPoseDisplay::reset()
                {
                    Display::reset();
                    clear();
                }

            }
        }
    }
}

