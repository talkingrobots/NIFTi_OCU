/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_VALIDATE_FLOAT_H
#define RVIZ_VALIDATE_FLOAT_H

#include <cmath>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/array.hpp>

namespace rviz
{

    class FloatValidator
    {
    private:
        FloatValidator();

    public:

        static inline bool validateFloats(float val)
        {
            return !(std::isnan(val) || std::isinf(val));
        }

        static inline bool validateFloats(double val)
        {
            return !(std::isnan(val) || std::isinf(val));
        }

        static inline bool validateFloats(const Ogre::Vector3& vec)
        {
            bool valid = true;
            valid = valid && validateFloats(vec.x);
            valid = valid && validateFloats(vec.y);
            valid = valid && validateFloats(vec.z);
            return valid;
        }

        static inline bool validateFloats(const Ogre::Quaternion& quat)
        {
            bool valid = true;
            valid = valid && validateFloats(quat.x);
            valid = valid && validateFloats(quat.y);
            valid = valid && validateFloats(quat.z);
            valid = valid && validateFloats(quat.w);
            return valid;
        }

        static inline bool validateFloats(const sensor_msgs::CameraInfo& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.D);
            valid = valid && validateFloats(msg.K);
            valid = valid && validateFloats(msg.R);
            valid = valid && validateFloats(msg.P);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::Point& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.x);
            valid = valid && validateFloats(msg.y);
            valid = valid && validateFloats(msg.z);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::Point32& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.x);
            valid = valid && validateFloats(msg.y);
            valid = valid && validateFloats(msg.z);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::Vector3& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.x);
            valid = valid && validateFloats(msg.y);
            valid = valid && validateFloats(msg.z);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::Twist& twist)
        {
            bool valid = true;
            valid = valid && validateFloats(twist.linear);
            valid = valid && validateFloats(twist.angular);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::Quaternion& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.x);
            valid = valid && validateFloats(msg.y);
            valid = valid && validateFloats(msg.z);
            valid = valid && validateFloats(msg.w);
            return valid;
        }

        static inline bool validateFloats(const std_msgs::ColorRGBA& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.r);
            valid = valid && validateFloats(msg.g);
            valid = valid && validateFloats(msg.b);
            valid = valid && validateFloats(msg.a);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::PointStamped& msg)
        {
            return validateFloats(msg.point);
        }

        static inline bool validateFloats(const geometry_msgs::Pose& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.position);
            valid = valid && validateFloats(msg.orientation);
            return valid;
        }

        static inline bool validateFloats(const geometry_msgs::PoseStamped& msg)
        {
            return validateFloats(msg.pose);
        }
        
        static inline bool validateFloats(const geometry_msgs::PoseArray& msg)
        {
            return validateFloats(msg.poses);
        }

        static inline bool validateFloats(const nav_msgs::OccupancyGrid& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.info.resolution);
            valid = valid && validateFloats(msg.info.origin);
            return valid;
        }

        static inline bool validateFloats(const nav_msgs::Path& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.poses);
            return valid;
        }

        static inline bool validateFloats(const visualization_msgs::Marker& msg)
        {
            bool valid = true;
            valid = valid && validateFloats(msg.pose);
            valid = valid && validateFloats(msg.scale);
            valid = valid && validateFloats(msg.color);
            valid = valid && validateFloats(msg.points);
            return valid;
        }

        template<typename T>
        static inline bool validateFloats(const std::vector<T>& vec)
        {
            typedef std::vector<T> VecType;
            typename VecType::const_iterator it = vec.begin();
            typename VecType::const_iterator end = vec.end();
            for (; it != end; ++it)
            {
                if (!validateFloats(*it))
                {
                    return false;
                }
            }

            return true;
        }

        template<typename T, size_t N>
        static inline bool validateFloats(const boost::array<T, N>& arr)
        {
            typedef boost::array<T, N> ArrType;
            typename ArrType::const_iterator it = arr.begin();
            typename ArrType::const_iterator end = arr.end();
            for (; it != end; ++it)
            {
                if (!validateFloats(*it))
                {
                    return false;
                }
            }

            return true;
        }

    };

} // namespace rviz

#endif // RVIZ_VALIDATE_FLOAT_H
