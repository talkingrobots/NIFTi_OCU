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


#ifndef RVIZ_GRID_CELLS_DISPLAY_H
#define RVIZ_GRID_CELLS_DISPLAY_H

#include <OGRE/OgreColourValue.h>

#include "Displays/Display.h"

#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/shared_ptr.hpp>

namespace ogre_tools
{
    class PointCloud;
}

namespace Ogre
{
    class ColourValue;
    class SceneNode;
    class ManualObject;
}

namespace rviz
{

/**
 * \class GridCellsDisplay
 * \brief Displays a nav_msgs::GridCells message
 */
class GridCellsDisplay : public Display
{
public:
  GridCellsDisplay( const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue );
  virtual ~GridCellsDisplay();

  void setTopic( const std::string& topic );

  void setColor( const Ogre::ColourValue& color );
  const Ogre::ColourValue& getColor() const;

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nav_msgs::GridCells::ConstPtr& msg);
  void processMessage(const nav_msgs::GridCells::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  Ogre::ColourValue color;

  Ogre::SceneNode* scene_node_;
  ogre_tools::PointCloud* cloud_;

  message_filters::Subscriber<nav_msgs::GridCells> sub_;
  tf::MessageFilter<nav_msgs::GridCells> tf_filter_;
  nav_msgs::GridCells::ConstPtr current_message_;

  uint32_t messages_received_;

  static const std::string ROS_TOPIC;
};

} // namespace rviz

#endif /* RVIZ_GRID_CELLS_DISPLAY_H */

