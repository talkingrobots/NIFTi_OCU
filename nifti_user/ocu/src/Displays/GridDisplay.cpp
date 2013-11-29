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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "ogre_tools/grid.h"

#include "NIFTiConstants.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Displays/GridDisplay.h"


namespace rviz
{

    GridDisplay::GridDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameMgr, updateQueue, threadQueue)
    , color(0.5f, 0.5f, 0.5f, 0.5f)
    , plane_(XY)
    {
        offset_ = Ogre::Vector3::ZERO;

        scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();
        grid_ = new ogre_tools::Grid(sceneMgr, scene_node_, ogre_tools::Grid::Lines, 10, 1.0f, 0.03f, color);
        grid_->getSceneNode()->setVisible(false);

        setCellCount(100);
        setFrame("<Fixed Frame>"); //Grid.Reference\ Frame=<Fixed Frame>
        setPlane(XY);
    }

    GridDisplay::~GridDisplay()
    {
        delete grid_;
        sceneMgr->destroySceneNode(scene_node_);
    }

    void GridDisplay::onEnable()
    {
        grid_->getSceneNode()->setVisible(true);
    }

    void GridDisplay::onDisable()
    {
        grid_->getSceneNode()->setVisible(false);
    }

    void GridDisplay::setCellSize(float size)
    {
        grid_->setCellLength(size);

        causeRender();
    }

    void GridDisplay::setCellCount(uint32_t count)
    {
        grid_->setCellCount(count);

        causeRender();
    }

    void GridDisplay::setColor(const Ogre::ColourValue& color)
    {
        this->color = color;
        grid_->setColor(color);

		causeRender();
    }

    const Ogre::ColourValue& GridDisplay::getColor() const
    {
        return color;
    }

    void GridDisplay::setLineWidth(float width)
    {
        grid_->setLineWidth(width);

        causeRender();
    }

    void GridDisplay::setStyle(int style)
    {
        grid_->setStyle((ogre_tools::Grid::Style)style);

        causeRender();
    }

    void GridDisplay::setHeight(uint32_t height)
    {
        grid_->setHeight(height);

        causeRender();
    }

    void GridDisplay::setOffset(const Ogre::Vector3& offset)
    {
        offset_ = offset;

        Ogre::Vector3 ogre_offset = offset;

        grid_->getSceneNode()->setPosition(ogre_offset);

        causeRender();
    }

    void GridDisplay::setPlane(int plane)
    {
        plane_ = (Plane) plane;

        if (plane_ == XY)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(1.0f, 0.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, -1.0f), Ogre::Vector3(0.0f, 1.0f, 0.0f)));
  }
  else if (plane_ == XZ)
  {
    grid_->getSceneNode()->setOrientation(1.0f, 0.0f, 0.0f, 0.0f);
  }
  else if (plane_ == YZ)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(0.0f, -1.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, 1.0f), Ogre::Vector3(1.0f, 0.0f, 0.0f)));
  }

        causeRender();
    }

    void GridDisplay::setFrame(const std::string& frame)
    {
        frame_ = frame;
    }

    void GridDisplay::update(float dt, float ros_dt)
    {
        std::string frame = frame_;
        if (frame == eu::nifti::ocu::NIFTiConstants::FIXED_FRAME_STRING)
        {
            frame = fixed_frame_;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (frameTransformer->getTransform(frame, ros::Time(), position, orientation))
        {
            scene_node_->setPosition(position);
            scene_node_->setOrientation(orientation);
            setStatus("Transform", eu::nifti::ocu::STATUS_LEVEL_OK, "Transform OK");
        } else
        {
            std::string error;
            if (frameTransformer->transformHasProblems(frame, ros::Time(), error))
            {
                setStatus("Transform", eu::nifti::ocu::STATUS_LEVEL_ERROR, error);
            } else
            {
                std::stringstream ss;
                ss << "Could not transform from [" << frame << "] to [" << fixed_frame_ << "]";
                setStatus("Transform", eu::nifti::ocu::STATUS_LEVEL_ERROR, ss.str());
            }
        }
    }

} // namespace rviz
