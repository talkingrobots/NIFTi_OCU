// Benoit 2012-05-21



#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>

#include <std_msgs/Header.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/exceptions.h>

#include "IVisualizationManager.h"
#include "NIFTiROSUtil.h"
#include "VirtualSceneManager.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Tools/GoToTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            namespace tools
            {

                const std::string ROS_TOPIC_NAV_GOAL = "/move_base_simple/goal";

                GoToTool::GoToTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, eu::nifti::ocu::VirtualSceneManager* virtualSceneMgr)
                : StandardOCUTool(id, name, iconFileName, publisherViewControl)
                , virtualSceneMgr(virtualSceneMgr)
                {
                    setCursorVirtualScene("CursorGoTo", 0, CURSOR_SIZE - 1);

                    publisherNavGoal = NIFTiROSUtil::getNodeHandle()->advertise<geometry_msgs::PoseStamped > (ROS_TOPIC_NAV_GOAL, 1);
                }

                bool GoToTool::onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    Ogre::Vector3 posMouse = getOgrePositionFromMouseCoordinates(evt.vizMgr->getViewport(), evt.evt.GetX(), evt.evt.GetY());

                    if (posMouse.x == 0 && posMouse.y == 0)
                    {
                        // This means that no position was found: the user clicked in the omnicam or in the sky
                        return false;
                    }

                    tf::Stamped<tf::Point> posMouseTransformed(tf::Point(posMouse.x, posMouse.y, posMouse.z), ros::Time(), virtualSceneMgr->getFixedFrame());

                    // Position of the robot
                    geometry_msgs::PointStamped pointStamped_Robot_wrt_base_link;
                    pointStamped_Robot_wrt_base_link.header.frame_id = "/base_link";
                    pointStamped_Robot_wrt_base_link.point.x = 0;
                    pointStamped_Robot_wrt_base_link.point.y = 0;
                    pointStamped_Robot_wrt_base_link.point.z = 0;

                    geometry_msgs::PointStamped pointStamped_Robot_wrt_map;

                    try
                    {
                        rviz::FrameTransformer::getTFClient()->transformPoint(virtualSceneMgr->getFixedFrame(), pointStamped_Robot_wrt_base_link, pointStamped_Robot_wrt_map);

                        // Calculates the angle between the robot and its goal (so that it has the perfect orientation if it goes in a straight line)
                        double angle = atan2(posMouseTransformed.y() - pointStamped_Robot_wrt_map.point.y, posMouseTransformed.x() - pointStamped_Robot_wrt_map.point.x);

                        publishGoal(posMouseTransformed.x(), posMouseTransformed.y(), angle);
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("%s", ex.what());
                    }

                    return false; // Does not switch to the default tool
                }

                void GoToTool::publishGoal(double x, double y, double theta)
                {
                    tf::Quaternion quat;
                    quat.setRPY(0.0, 0.0, theta);

                    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose > (tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), virtualSceneMgr->getFixedFrame());

                    geometry_msgs::PoseStamped goal;
                    tf::poseStampedTFToMsg(p, goal);

                    //        ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
                    //                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                    //                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);

                    publisherNavGoal.publish(goal);
                }

            }
        }
    }
}
