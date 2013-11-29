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

#ifndef RVIZ_DISPLAY_H
#define RVIZ_DISPLAY_H

#include <string>
#include <map>

#include <boost/function.hpp>

#include <ros/node_handle.h>

#include "AnnotatedStatusLevel.h"

namespace Ogre
{
    class SceneManager;
}

namespace ros
{
    class CallbackQueueInterface;
}

namespace rviz
{

    class FrameTransformer;

    /**
     * Provides a common interface for the visualization panel to interact with,
     * so that new displays can be added without the visualization panel knowing
     * anything about them.
     * Benoit: Eventually, maybe this could be split into GenericDisplay, ROSDisplay, CASTDisplay
     */
    class Display
    {
    public:
        Display(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue);
        virtual ~Display();

        /**
         * \brief Enable this display
         * @param force If false, does not re-enable if this display is already enabled.  If true, it does.
         */
        void enable(bool force = false);
        /**
         * \brief Disable this display
         * @param force If false, does not re-disable if this display is already disabled.  If true, it does.
         */
        void disable(bool force = false);

        bool isEnabled() const;
        
        void setEnabled(bool enable, bool force = false);

        const std::string& getName() const;

        const std::string& getTopic() const;

        //virtual void setTopic(const std::string& topic) = 0;
        
        /**
         * \brief Called periodically by the visualization panel
         * @param dt Wall-clock time, in seconds, since the last time the update list was run through.
         */
        virtual void update(float wall_dt, float ros_dt) = 0;
        
        ///
        /**
         * \brief Set the callback used for causing a render to happen
         * @param func a void(void) function that will cause a render to happen from the correct thread
         */
        void setRenderCallback(boost::function<void () > func);

        /// Set the callback used to lock the renderer
        void setLockRenderCallback(boost::function<void () > func);
        /// Set the callback used to unlock the renderer
        void setUnlockRenderCallback(boost::function<void () > func);

        /**
         * \brief Set the fixed frame of this display.  This is a frame id which should generally be the top-level frame being broadcast through TF
         * @param frame The fixed frame
         */
        void setFixedFrame(const std::string& frame);

        /**
         * \brief Called from within setFixedFrame, notifying child classes that the fixed frame has changed
         */
        virtual void fixedFrameChanged() = 0;

        const eu::nifti::ocu::AnnotatedStatusLevel& getStatus(const std::string& statusName) const;

        void setStatus(const std::string statusName, const eu::nifti::ocu::StatusLevel& level, const std::string& msg);
        //void setStatus(const std::string statusName, const AnnotatedStatusLevel status);        

        void deleteStatus(const std::string& name);

        void clearStatuses();

        /**
         * \brief Called to tell the display to clear its state
         */
        virtual void reset();

    protected:
        /// Derived classes override this to do the actual work of enabling themselves
        virtual void onEnable() = 0;
        /// Derived classes override this to do the actual work of disabling themselves
        virtual void onDisable() = 0;

        /**
         * Causes the scene to be rendered. This does not immediately cause a
         * render; instead, one is queued and happens at the next run through
         * the event loop.
         */
        void causeRender();

        /// Lock the renderer
        void lockRender();
        /// Unlock the renderer
        void unlockRender();

        /**
         / Name given to easily identify the display
         */
        const std::string name;

        /**
         * Topic on which ROS listens
         */
        std::string topic;

        /**
         * Indicates whether the display is shown on the screen
         */
        bool enabled;

        /**
         * Collection of statuses for various aspects of the display
         */
        std::map<std::string, eu::nifti::ocu::AnnotatedStatusLevel> statuses;

        /**
         * Ogre manager for the 3D scene
         */
        Ogre::SceneManager* sceneMgr;

        FrameTransformer* frameTransformer;

        ros::NodeHandle update_nh_;
        ros::NodeHandle threaded_nh_;

        std::string fixed_frame_; ///< The frame we should transform all fixed data into

        boost::function<void () > render_callback_; ///< Render callback
        boost::function<void () > render_lock_; ///< Render lock callback
        boost::function<void () > render_unlock_; ///< Render unlock callback

    };

} // namespace rviz

#endif
