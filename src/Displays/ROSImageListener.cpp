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

// Benoit 2012-10-10 Changed the mutex for a re-entrant one, to use clear when the topic/transport type is incorrect

#include <tf/tf.h>

#include <sensor_msgs/image_encodings.h>

#include <image_transport/exception.h>

#include <OGRE/OgreTextureManager.h>

#include "Displays/ROSImageListener.h"

namespace rviz
{
    // Because it's static, the image must be initialized here
    Ogre::Image ROSImageListener::EMPTY_IMAGE = Ogre::Image();

    ROSImageListener::ROSImageListener(const ros::NodeHandle& nh)
    : nodeHandle(nh)
    , imageTransport(nh)
    , transport_type_("raw")
    , hasReceivedNewImage(false)
    , tf_client_(0)
    , image_count_(0)
    {
        ROSImageListener::EMPTY_IMAGE.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        latestImageLoaded = ROSImageListener::EMPTY_IMAGE;

        static uint32_t count = 0;
        std::stringstream ss;
        ss << "ROSImageListener" << count++;
        texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, ROSImageListener::EMPTY_IMAGE);
    }

    ROSImageListener::~ROSImageListener()
    {
        latestImageMsgReceived.reset();
    }

    void ROSImageListener::clear()
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);

        sub_->unsubscribe(); // Benoit: Added 2012-10-10

        texture_->unload();
        texture_->loadImage(ROSImageListener::EMPTY_IMAGE);

        hasReceivedNewImage = false; // Ensures that an old image will not get loaded
        latestImageMsgReceived.reset();

        if (tf_filter_)
        {
            tf_filter_->clear();
        }

        image_count_ = 0;
    }

    void ROSImageListener::setFrame(const std::string& frame, tf::TransformListener* tf_client)
    {
        this->tf_client_ = tf_client;
        this->frame_ = frame;
        setTopic(topic_); // Could throw a image_transport::TransportLoadException
    }

    void ROSImageListener::setTopic(const std::string& topic)
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);

        // Must reset the current image here because image_transport will unload the plugin as soon as we unsubscribe,
        // which removes the code segment necessary for the shared_ptr's deleter to exist!
        latestImageMsgReceived.reset();

        hasReceivedNewImage = false; // Ensures that we don't try to display an image when there is none.

        topic_ = topic;
        tf_filter_.reset();

        if (!sub_)
        {
            sub_.reset(new image_transport::SubscriberFilter());
        }

        if (!topic.empty())
        {
            try
            {
                sub_->subscribe(imageTransport, topic, 1, image_transport::TransportHints(transport_type_));
            }
            catch (image_transport::TransportLoadException& ex)
            {
                clear();
                throw;
            }

            if (frame_.empty())
            {
                sub_->registerCallback(boost::bind(&ROSImageListener::callback, this, _1));
            }
            else
            {
                ROS_ASSERT(tf_client_);
                tf_filter_.reset(new tf::MessageFilter<sensor_msgs::Image > (*sub_, (tf::Transformer&) * tf_client_, frame_, 2, nodeHandle));
                tf_filter_->registerCallback(boost::bind(&ROSImageListener::callback, this, _1));
            }
        }
    }

    void ROSImageListener::setTransportType(const std::string& transport_type)
    {
        transport_type_ = transport_type;
        setTopic(topic_); // Could throw a image_transport::TransportLoadException
    }

    const std::string& ROSImageListener::getTransportType() const
    {
        return transport_type_;
    }

    void ROSImageListener::getAvailableTransportTypes(V_string& types)
    {
        types.push_back("raw");

        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        ros::master::V_TopicInfo::iterator it = topics.begin();
        ros::master::V_TopicInfo::iterator end = topics.end();
        for (; it != end; ++it)
        {
            const ros::master::TopicInfo& ti = *it;
            if (ti.name.find(topic_) == 0 && ti.name != topic_)
            {
                std::string type = ti.name.substr(topic_.size() + 1);
                if (type.find('/') == std::string::npos)
                {
                    types.push_back(type);
                }
            }
        }
    }

    const sensor_msgs::Image::ConstPtr& ROSImageListener::getLatestImageMsgReceived()
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);

        return latestImageMsgReceived;
    }

    const Ogre::Image& ROSImageListener::getLatestImageLoaded()
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);

        return latestImageLoaded;
    }

    Ogre::Image ROSImageListener::getOgreImageFromTexture()
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);

        // THIS IS A STUPID HACK. SEE DETAILS AT BOTTOM OF FILE
        //printf("Texture size: %i", texture_->getSize());
        if (texture_->getSize() == 5120000)
        {
            throw "Image too big. See Ticket #198";
        }

        Ogre::Image i;
        texture_->convertToImage(i, false);
        return i;
    }

    bool ROSImageListener::update()
    {
        // Todo Benoit Establish a treshold so that after x seconds, it displays "No Image" or something else, like a thumbnail of the last image or a timestamp on the last image

        // Returns false if there is nothing to update
        if (this->hasReceivedNewImage == false)
        {
            return false;
        }

        // Loads the data, from the latest received message

        void* data_ptr = (void*) &latestImageMsgReceived->data[0];
        uint32_t data_size = latestImageMsgReceived->data.size();

        Ogre::PixelFormat format;

        if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::RGB8)
        {
            format = Ogre::PF_BYTE_RGB;
        }
        else if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::RGBA8)
        {
            format = Ogre::PF_BYTE_RGBA;
        }
        else if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::BGRA8)
        {
            format = Ogre::PF_BYTE_BGRA;
        }
        else if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::BGR8)
        {
            format = Ogre::PF_BYTE_BGR;
        }
        else if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::MONO8)
        {
            format = Ogre::PF_BYTE_L;
        }
        else if (latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                latestImageMsgReceived->encoding == sensor_msgs::image_encodings::MONO16)
        {
            format = Ogre::PF_BYTE_L;

            // downsample manually to 8-bit, because otherwise the lower 8-bits are simply removed
            std::vector<uint8_t> buffer;
            buffer.resize(latestImageMsgReceived->data.size() / 2);
            data_size = buffer.size();
            data_ptr = (void*) &buffer[0];
            for (size_t i = 0; i < data_size; ++i)
            {
                uint16_t s = latestImageMsgReceived->data[2 * i] << 8 | latestImageMsgReceived->data[2 * i + 1];
                float val = (float) s / std::numeric_limits<uint16_t>::max();
                buffer[i] = val * 255;
            }
        }
        else if (latestImageMsgReceived->encoding.find("bayer") == 0)
        {
            format = Ogre::PF_BYTE_L;
        }
        else
        {
            throw UnsupportedImageEncoding(latestImageMsgReceived->encoding);
        }


        // TODO: Support different steps/strides

        Ogre::DataStreamPtr pixel_stream;
        pixel_stream.bind(new Ogre::MemoryDataStream(data_ptr, data_size));

        try
        {
            latestImageLoaded.loadRawData(pixel_stream, latestImageMsgReceived->width, latestImageMsgReceived->height, format);
        }
        catch (Ogre::Exception& e)
        {
            // TODO: signal error better
            ROS_ERROR("Error loading image: %s", e.what());
            return false;
        }

        // Removes the old texture/image and show the new one
        texture_->unload();
        texture_->loadImage(latestImageLoaded);

        this->hasReceivedNewImage = false;

        return true;
    }

    /**
     * Stores a new image (update must be called to display it)
     * @param msg
     */
    void ROSImageListener::callback(const sensor_msgs::Image::ConstPtr& msg)
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);
        latestImageMsgReceived = msg;
        hasReceivedNewImage = true;

        ++image_count_;
    }

}


//
//http://poor3d.googlecode.com/svn-history/r34/trunk/Poor3D/Engine/MemoryManager/nedmalloc.h
//
//
///* Gets the usable size of an allocated block. Note this will always be bigger than what was
//asked for due to rounding etc. Optionally returns 1 in isforeign if the block came from the
//system allocator - note that there is a small (>0.01%) but real chance of segfault on non-Windows
//systems when passing non-nedmalloc blocks if you don't use USE_MAGIC_HEADERS.
//*/
//NEDMALLOCEXTSPEC NEDMALLOCNOALIASATTR size_t nedblksize(int *RESTRICT isforeign, void *RESTRICT mem) THROWSPEC;
//
//
//
//
//ISO C (and thus also C++) allows you to catch and handle the SIGSEGV
//signal (SEGV is short for "segmentation violation"). However, the only
//sane reason to do so is to save work before aborting the program; the
//behavior of a program which ignores SIGSEGV (i.e., one which continues
//running after SIGSEGV) is undefined.