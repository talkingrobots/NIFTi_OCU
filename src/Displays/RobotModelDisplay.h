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

#ifndef RVIZ_ROBOT_MODEL_DISPLAY_H
#define RVIZ_ROBOT_MODEL_DISPLAY_H

#include <map>

#include <OGRE/OgreVector3.h>

#include <nifti_robot_driver_msgs/RobotStatus.h>

#include "Displays/Display.h"

namespace Ogre
{
    class Entity;
    class SceneNode;
}

namespace ogre_tools
{
    class Axes;
}

namespace rviz
{

    class Robot;
    class RobotLink;

    /**
     * \class RobotModelDisplay
     * \brief Uses a robot xml description to display the pieces of a robot at the transforms broadcast by rosTF
     */
    class RobotModelDisplay : public Display
    {
    public:
        RobotModelDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        virtual ~RobotModelDisplay();

        /**
         * \brief Set the robot description parameter
         * @param description_param The ROS parameter name which contains the robot xml description
         */
        void setRobotDescription(const std::string& description_param);

        virtual void update(float wall_dt, float ros_dt);

        /**
         * \brief Set whether the visual mesh representation should be displayed
         * @param visible
         */
        void setVisualVisible(bool visible);

        /**
         * \brief Set whether the collision representation should be displayed
         * @param visible
         */
        void setCollisionVisible(bool visible);

        /**
         * \brief Set the rate at which we request new transforms from libTF
         * @param rate The rate, in seconds
         */
        void setUpdateRate(float rate);

        const std::string& getRobotDescription()
        {
            return description_param_;
        }

        float getUpdateRate()
        {
            return update_rate_;
        }
        bool isVisualVisible();
        bool isCollisionVisible();

        float getAlpha()
        {
            return alpha_;
        }
        void setAlpha(float alpha);

        const std::string& getTFPrefix()
        {
            return tf_prefix_;
        }
        void setTFPrefix(const std::string& prefix);

        void clear();

        // Overrides from Display

        virtual void fixedFrameChanged()
        {
        }
        virtual void reset();

        static inline double normalizeValue(const double value)
        {
            if (value < 0)
                return 0;
            else if (value > 1) 
                return 1;
            else
                return value;
        }

    protected:

        /**
         * \brief Loads a URDF from our #description_param_, iterates through the links and loads any necessary models
         */
        void load();

        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        void updatePlatformColor();
        void updateFlippersColors();
        void updateFlipperColor(RobotLink* flipper, double current);
        
        void getPlatformAngles(double& pitch, double& roll);

        std::string description_param_; ///< ROS parameter that contains the robot xml description

        Robot* robot_; ///< Handles actually drawing the robot

        bool has_new_transforms_; ///< Callback sets this to tell our update function it needs to update the transforms

        float time_since_last_transform_;
        float update_rate_;
        float alpha_;
        std::string tf_prefix_;

        std::string robot_description_;
    };

} // namespace rviz

#endif

