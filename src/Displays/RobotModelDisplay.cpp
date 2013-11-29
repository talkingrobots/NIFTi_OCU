/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

//rostopic pub -1 /currents nifti_robot_driver_msgs/Currents -- 1 1 0 0.5 1 2  
//rosrun rviz rviz _/rviz/robot_description:="/nifti_systems/bleeding-edge/stacks/nifti_user/ocu/media/robot_model/NIFTi.urdf"

#include <cmath>

#include "Robot/Robot.h"
#include "Robot/TFLinkUpdater.h"
#include "Robot/RobotLink.h"

#include <urdf/model.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <nifti_robot_driver_msgs/CurrentsStamped.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "NIFTiROSUtil.h"
#include "RobotsStatusesManager.h"

#include "Displays/RobotModelDisplay.h"

namespace rviz
{
    const std::string ROBOT_DESCRIPTION_PARAMETER = "/ugv3DModelDescription";

    const double MAX_CURRENT_FLIPPERS = 1.625;
    const double MAX_PITCH = 0.7; // Value found experimentally by tilting the platform
    const double MAX_ROLL = 0.9; // Value found experimentally by tilting the platform
    //rostopic pub /set_flippers_torque nifti_robot_driver_msgs/FlippersTorque -1  3.0 3.0

    void linkUpdaterStatusFunction(eu::nifti::ocu::StatusLevel level, const std::string& link_name, const std::string& text, RobotModelDisplay* display)
    {
        display->setStatus(link_name, level, text);
    }

    RobotModelDisplay::RobotModelDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
    , description_param_(eu::nifti::ocu::NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace() + ROBOT_DESCRIPTION_PARAMETER)
    , has_new_transforms_(false)
    , time_since_last_transform_(0.0f)
    , update_rate_(0.0f)
    {
        robot_ = new Robot(sceneMgr, "Robot: " + name);

        setVisualVisible(true);
        setCollisionVisible(false);

        setAlpha(1.0f);
    }

    RobotModelDisplay::~RobotModelDisplay()
    {
        delete robot_;
    }

    void RobotModelDisplay::setAlpha(float alpha)
    {
        alpha_ = alpha;

        robot_->setAlpha(alpha_);
    }

    void RobotModelDisplay::setRobotDescription(const std::string& description_param)
    {
        description_param_ = description_param;

        if (isEnabled())
        {
            load();
            causeRender();
        }
    }

    void RobotModelDisplay::setVisualVisible(bool visible)
    {
        robot_->setVisualVisible(visible);

        causeRender();
    }

    void RobotModelDisplay::setCollisionVisible(bool visible)
    {
        robot_->setCollisionVisible(visible);

        causeRender();
    }

    void RobotModelDisplay::setUpdateRate(float rate)
    {
        update_rate_ = rate;

        causeRender();
    }

    void RobotModelDisplay::setTFPrefix(const std::string& prefix)
    {
        tf_prefix_ = prefix;

        causeRender();
    }

    bool RobotModelDisplay::isVisualVisible()
    {
        return robot_->isVisualVisible();
    }

    bool RobotModelDisplay::isCollisionVisible()
    {
        return robot_->isCollisionVisible();
    }

    void RobotModelDisplay::load()
    {
        std::string content;

        // If the description parameter is not found directly, search in the parameter tree
        if (!update_nh_.getParam(description_param_, content))
        {
            std::string loc;
            if (update_nh_.searchParam(description_param_, loc))
            {
                update_nh_.getParam(loc, content);
            }
            else
            {
                clear();

                std::stringstream ss;
                ss << "Parameter [" << description_param_ << "] does not exist, and was not found by searchParam()";
                setStatus("URDF", eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
                return;
            }
        }

        //ROS_INFO_STREAM("content: " << content);

        if (content.empty())
        {
            clear();
            setStatus("URDF", eu::nifti::ocu::STATUS_LEVEL_ERROR, "URDF is empty");
            return;
        }

        if (content == robot_description_)
        {
            return;
        }

        robot_description_ = content;

        TiXmlDocument doc;
        doc.Parse(robot_description_.c_str());
        if (!doc.RootElement())
        {
            clear();
            setStatus("URDF", eu::nifti::ocu::STATUS_LEVEL_ERROR, "URDF failed XML parse");
            return;
        }

        urdf::Model descr;
        if (!descr.initXml(doc.RootElement()))
        {
            clear();
            setStatus("URDF", eu::nifti::ocu::STATUS_LEVEL_ERROR, "URDF failed Model parse");
            return;
        }

        setStatus("URDF", eu::nifti::ocu::STATUS_LEVEL_OK, "URDF parsed OK");
        robot_->load(doc.RootElement(), descr);
        robot_->update(TFLinkUpdater(frameTransformer, boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this), tf_prefix_));
    }

    void RobotModelDisplay::onEnable()
    {
        load();
        robot_->setVisible(true);
    }

    void RobotModelDisplay::onDisable()
    {
        robot_->setVisible(false);
        clear();
    }

    void RobotModelDisplay::update(float wall_dt, float ros_dt)
    {
        time_since_last_transform_ += wall_dt;

        bool update = update_rate_ < 0.0001f || time_since_last_transform_ >= update_rate_;

        if (has_new_transforms_ || update)
        {
            robot_->update(TFLinkUpdater(frameTransformer, boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this), tf_prefix_));
            updatePlatformColor();
            updateFlippersColors();
            causeRender();

            has_new_transforms_ = false;
            time_since_last_transform_ = 0.0f;
        }
    }

    void RobotModelDisplay::clear()
    {
        robot_->clear();
        clearStatuses();
        robot_description_.clear();
    }

    void RobotModelDisplay::reset()
    {
        Display::reset();
        has_new_transforms_ = true;
    }

    void RobotModelDisplay::updatePlatformColor()
    {
        double pitch, roll;
        getPlatformAngles(pitch, roll);
        
        // Normalizes between 0 and 1;            
        double pitchAbsolute = (pitch >= 0 ? pitch : -pitch);
        double rollAbsolute = (roll >= 0 ? roll : -roll);
        double pitchNormalized = pitchAbsolute / MAX_PITCH;
        double rollNormalized = rollAbsolute / MAX_ROLL;
        double riskNormalized = normalizeValue(pitchNormalized + rollNormalized); // Sum of both

        // Maps the current to a graduation of green -> yellow -> red
        double red = normalizeValue(2 * riskNormalized);
        double green = normalizeValue(2 * (1 - riskNormalized));

        // Makes the color a little bit darker (it looks nicer)
        red *= 0.75;
        green *= 0.75;

        RobotLink* baseLink = robot_->getLink("base_link");

        baseLink->setColor(red, green, 0, 1);
    }

    void RobotModelDisplay::updateFlippersColors()
    {
        // TODO Read the error codes from the status message, to display the flippers as white
        // For now, I just ignore received messages if the model is not loaded

        const nifti_robot_driver_msgs::CurrentsStampedConstPtr& c = eu::nifti::ocu::RobotsStatusesManager::getFlippersCurrents();

        // Ensures that there is a message about the flippers that has arrived
        if (!c) return;
        if (!(c.get())) return;

        // Todo: Read these parameters. They are defined in Francis' documentation
        RobotLink* fl_flipper = robot_->getLink("front_left_flipper");
        RobotLink* fr_flipper = robot_->getLink("front_right_flipper");
        RobotLink* rl_flipper = robot_->getLink("rear_left_flipper");
        RobotLink* rr_flipper = robot_->getLink("rear_right_flipper");

        if (!fl_flipper) return;
        if (!fr_flipper) return;
        if (!rl_flipper) return;
        if (!rr_flipper) return;

        updateFlipperColor(fl_flipper, c.get()->frontLeft);
        updateFlipperColor(fr_flipper, c.get()->frontRight);
        updateFlipperColor(rl_flipper, c.get()->rearLeft);
        updateFlipperColor(rr_flipper, c.get()->rearRight);

    }

    void RobotModelDisplay::updateFlipperColor(RobotLink* flipper, double current)
    {
        if (current == 0) // means that the flipper is disabled
        {
            flipper->setColor(0, 0, 0, 1);
        }
        else
        {
            // Normalizes the current between 0 and 1;            
            double currentAbsolute = (current >= 0 ? current : -current);
            double currentNormalized = currentAbsolute / MAX_CURRENT_FLIPPERS;

            // Maps the current to a graduation of green -> yellow -> red
            double red = normalizeValue(2 * currentNormalized);
            double green = normalizeValue(2 * (1 - currentNormalized));

            // Makes the color a little bit darker (it looks nicer)
            red *= 0.75;
            green *= 0.75;

            flipper->setColor(red, green, 0, 1);
        }
    }

    void RobotModelDisplay::getPlatformAngles(double& pitch, double& roll)
    {
        //FrameTransfomrer
        //bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation);

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!frameTransformer->getTransform("base_link", ros::Time(0), position, orientation))
        {
            ROS_DEBUG("Error transforming from frame 'map' to frame 'base_link'");
        }

        // These formulas come from the Rotation Cook Book, formula 290
        double q0 = orientation.w;
        double q1 = orientation.x;
        double q2 = orientation.y;
        double q3 = orientation.z;
        
        double angle1 = atan2( 2*q2*q3 + 2*q0*q1, pow(q3,2)-pow(q2,2)-pow(q1,2)+pow(q0,2) );
        double angle2 = -asin( 2*q1*q3 - 2*q0*q2 );
        //double angle3 = atan2( 2*q1*q2 + 2*q0*q3, pow(q1,2)+pow(q0,2)-pow(q3,2)-pow(q2,2) );
        
        //ROS_INFO_STREAM("NIFTi pitch? " << angle2 << " NIFTi roll? " << angle1);
            
        pitch = angle2;
        roll = angle1;
    }

} // namespace rviz


// Platform angle values
//With the arm
//
//Going up stairs: 
//
//-0.88 tipping point
//-0.7 safe
//
//Going down stairs:
//
//0.75 very safe
//1.2 tipping point
//
//Roll:
//
//-1.1 tipping point
//-0.9 safe