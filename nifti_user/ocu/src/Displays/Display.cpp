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

#include "Displays/Display.h"

namespace rviz
{

    Display::Display(const std::string& name, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : name(name)
    , enabled(false)
    , sceneMgr(sceneMgr)
    , frameTransformer(frameTransformer)
    {
        assert(sceneMgr != NULL);
        assert(frameTransformer != NULL);

        update_nh_.setCallbackQueue(updateQueue);
        threaded_nh_.setCallbackQueue(threadQueue);
    }

    Display::~Display()
    {

    }

    void Display::enable(bool force)
    {
        if (enabled && !force)
        {
            return;
        }

        enabled = true;

        onEnable();
    }

    void Display::disable(bool force)
    {
        if (!enabled && !force)
        {
            return;
        }

        enabled = false;

        onDisable();
    }

    bool Display::isEnabled() const
    {
        return enabled;
    }

    void Display::setEnabled(bool en, bool force)
    {
        if (en)
        {
            enable(force);
        } else
        {
            disable(force);
        }
    }

    const std::string& Display::getName() const
    {
        return name;
    }

    const std::string& Display::getTopic() const
    {
        return topic;
    }

    void Display::setRenderCallback(boost::function<void () > func)
    {
        render_callback_ = func;
    }

    void Display::setLockRenderCallback(boost::function<void () > func)
    {
        render_lock_ = func;
    }

    void Display::setUnlockRenderCallback(boost::function<void () > func)
    {
        render_unlock_ = func;
    }

    void Display::causeRender()
    {
        if (render_callback_)
        {
            render_callback_();
        }
    }

    void Display::lockRender()
    {
        if (render_lock_)
        {
            render_lock_();
        }
    }

    void Display::unlockRender()
    {
        if (render_unlock_)
        {
            render_unlock_();
        }
    }

    void Display::setFixedFrame(const std::string& frame)
    {
        fixed_frame_ = frame;

        fixedFrameChanged();
    }

    // Benoit todo throw an exception instead of using an assert
    const eu::nifti::ocu::AnnotatedStatusLevel& Display::getStatus(const std::string& statusName) const
    {
        // Apparently the 'at' method is not standard, so I changed the way in which I access the elements of the map
        //return statuses.at(statusName);

        std::map<std::string,eu::nifti::ocu::AnnotatedStatusLevel>::const_iterator it = statuses.find(statusName);

        assert( it != statuses.end() );

        return it->second;
    }

    //void Display::setStatus2(const std::string statusName, const AnnotatedStatusLevel status)

    void Display::setStatus(const std::string statusName, const eu::nifti::ocu::StatusLevel& level, const std::string& msg)
    {
        statuses.insert(std::pair<std::string, eu::nifti::ocu::AnnotatedStatusLevel > (statusName, eu::nifti::ocu::AnnotatedStatusLevel(level, msg)));
        
//        if(level == eu::nifti::ocu::STATUS_LEVEL_ERROR)
//        {
//            ROS_ERROR_STREAM("Error with display status '" << statusName << "': " << msg);
//        }
    }

    void Display::deleteStatus(const std::string& name)
    {
        statuses.erase(name);
    }

    void Display::clearStatuses()
    {
        statuses.clear();
    }

    void Display::reset()
    {
        clearStatuses();
    }

} // namespace rviz
