// Benoit 2013-01-11

#include <boost/foreach.hpp>

#include <ros/node_handle.h>

#include <EXIFReader_msgs/GetAnnotatedPicture.h>
#include <EXIFReader_msgs/GetAllAnnotatedPictures.h>

#include <nifti_pics_server_util/AnnotatedPicUtil.h>

#include "IRobotStatusListener.h"
#include "NIFTiROSUtil.h"

#include "InFieldPicsManager.h"

using namespace std;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const string TOPIC_NEW_PICS = "/inFieldPicsServer/new";
            const string TOPIC_GET_PICS = "/inFieldPicsServer/getAll";
            const string TOPIC_GET_PIC = "/inFieldPicsServer/get";
            const string TOPIC_MODIFY = "/inFieldPicsServer/modify";

            InFieldPicsManager* InFieldPicsManager::theInstance = NULL;

            InFieldPicsManager::InFieldPicsManager()
            {
            }

            InFieldPicsManager* InFieldPicsManager::instance()
            {
                if (theInstance == NULL)
                {
                    theInstance = new InFieldPicsManager();
                }
                return theInstance;
            }

            void InFieldPicsManager::init()
            {
                shutdownRequested = false;

                servGetPics = NIFTiROSUtil::getNodeHandle()->serviceClient<EXIFReader_msgs::GetAllAnnotatedPictures > (TOPIC_GET_PICS);
                servGetPic = NIFTiROSUtil::getNodeHandle()->serviceClient<EXIFReader_msgs::GetAnnotatedPicture > (TOPIC_GET_PIC);

                if (!servGetPics.exists() || !servGetPic.exists())
                {
                    ROS_WARN("The InFieldPicsServer does not appear to be running. No in-field pictures will be shown in the OCU.");

                    unInit();

                    return;
                }

                subscriberNewPics = NIFTiROSUtil::getNodeHandle()->subscribe(TOPIC_NEW_PICS, 100, &InFieldPicsManager::onNewInFieldPicMsgReceived, this);

                subscriberModification = NIFTiROSUtil::getNodeHandle()->subscribe(TOPIC_MODIFY, 100, &InFieldPicsManager::onModificationReceived, this);

                threadToCallListeners = boost::thread(&InFieldPicsManager::processNewMessages, this);
                // seems to start automatically
            }

            void InFieldPicsManager::unInit()
            {
                // Just as safety, since these objects are static, I unsubscribe here.
                subscriberNewPics.shutdown();

                servGetPics.shutdown();
                servGetPic.shutdown();

                {
                    boost::mutex::scoped_lock lock(mutexNewROSMessages);

                    shutdownRequested = true;
                    conditionNewROSMessages.notify_one();
                }

                threadToCallListeners.join();
            }

            std::list<const AnnotatedPicture*>* InFieldPicsManager::addListener(InFieldPicsManager::Listener* listener, bool returnExistingPictures)
            {
                assert(listener != NULL);

                boost::mutex::scoped_lock lock(mutexListeners);

                listeners.insert(listener);

                // Calls the callback function as if all pictures were new (they are new to this listener, as far as it is concerned)
                // Can return the list of all existing pictures, so that the listener is up-to-date
                if (returnExistingPictures)
                {
                    return getPictures();
                }
                else
                {
                    return NULL;
                }
            }

            void InFieldPicsManager::removeListener(InFieldPicsManager::Listener* listener)
            {
                assert(listener != NULL);

                boost::mutex::scoped_lock lock(mutexListeners);

                listeners.erase(listener);
            }

            std::list<const AnnotatedPicture*>* InFieldPicsManager::getPictures()
            {
                boost::mutex::scoped_lock lock(mutexPictures);

                std::list<const AnnotatedPicture*>* picturesToReturn = new std::list<const AnnotatedPicture*>(pictures.size());

                BOOST_FOREACH(const MapOfConstAnnotatedPictures::value_type& pair, pictures)
                {
                    picturesToReturn->push_back(pair.second);
                }

                return picturesToReturn;
            }

            /**
             * Upon the reception of a new picture, it just gets enqueued for processing by the worker thread
             * @param picture
             */
            void InFieldPicsManager::onNewInFieldPicMsgReceived(const EXIFReader_msgs::AnnotatedPicture* picture)
            {
                boost::mutex::scoped_lock lock(mutexNewROSMessages);

                newPictures.push(eu::nifti::misc::nifti_pics_server_util::AnnotatedPicUtil::cloneAnnotatedPicture(picture));
                conditionNewROSMessages.notify_one();
            }

            /**
             * This message comes directly from ROS, so we need to clone it. Otherwise, ROS will eventually delete it (through the boost shared pointer)
             * @param picture
             */
            void InFieldPicsManager::onNewInFieldPicMsgReceived(const EXIFReader_msgs::AnnotatedPictureConstPtr& picture)
            {
                onNewInFieldPicMsgReceived(picture.get());
            }

            void InFieldPicsManager::onModificationReceived(const EXIFReader_msgs::Modification& modificationMsg)
            {
                //ROS_INFO_STREAM("Received a modification message for [" << modificationMsg.filename << "]");

                boost::mutex::scoped_lock lock(mutexNewROSMessages);

                newModifications.push(modificationMsg);
                conditionNewROSMessages.notify_one();
            }

            const EXIFReader_msgs::AnnotatedPicture* InFieldPicsManager::getPicture(std::string filename)
            {
                boost::mutex::scoped_lock lock(mutexPictures);

                return pictures.at(filename);
            }

            /**
             * This runs from a boost worker thread
             */
            void InFieldPicsManager::processNewMessages()
            {
                //ROS_INFO("boost worker thread started");

                //1) Gets all pictures 

                EXIFReader_msgs::GetAllAnnotatedPictures::Request req;
                req.sendCompleteFiles = true;
                EXIFReader_msgs::GetAllAnnotatedPictures::Response res;
                servGetPics.call(req, res); // This is a blocking call!!!

                // This loop is somehow really fast
                BOOST_FOREACH(const EXIFReader_msgs::AnnotatedPicture& picture, res.pictures)
                {
                    onNewInFieldPicMsgReceived(&picture);
                }

                // 2) Wait for new pictures to be available and process them
                while (true)
                {
                    const EXIFReader_msgs::AnnotatedPicture* picture = NULL;
                    EXIFReader_msgs::Modification modification;

                    {
                        boost::mutex::scoped_lock lock(mutexNewROSMessages);

                        if (shutdownRequested) return;

                        // The thread will wait here until there are new pictures
                        if (newPictures.empty() == true && newModifications.empty() == true)
                        {
                            conditionNewROSMessages.wait(lock);
                        }

                        if (shutdownRequested) return;

                        assert(!newPictures.empty() || !newModifications.empty());

                        if(!newPictures.empty())
                        {
                            picture = newPictures.front();
                            newPictures.pop();
                        }
                        else // if(!newModifications.empty())
                        {
                            modification = newModifications.front();
                            newModifications.pop();
                        }
                        
                    }

                    if(picture != NULL)
                    {                    
                        processNewPicture(picture);
                    }
                    else
                    {
                        processNewModification(modification);
                    }
                    
                }

            }

            void InFieldPicsManager::processNewPicture(const EXIFReader_msgs::AnnotatedPicture* picture)
            {

                {
                    boost::mutex::scoped_lock lockListeners(mutexListeners);
                    boost::mutex::scoped_lock lockPictures(mutexPictures);

                    // Checks for duplicates
                    {
                        const EXIFReader_msgs::AnnotatedPicture* oldPicture = NULL;

                        try
                        {
                            oldPicture = pictures.at(picture->filename);

                            if (oldPicture->completeFile.size() == 0 && picture->completeFile.size() > 0)
                            {
                                // This is not a duplicate, but rather the reception of the actual picture data

                                // Need to do that because std::map::insert does not overwrite
                                pictures.erase(picture->filename);
                            }
                            else
                            {
                                cerr << "Received new AnnotatedPicture " << picture->filename << ", but it already exists. This situation should not happen. Maybe the In-Field Pictures Server was restarted. Restarting the OCU is recommended." << std::endl;
                                delete picture;
                                return;
                            }
                        }
                        catch (const std::out_of_range& ex)
                        {
                            // There is no picture with that filename, so everything is good.
                        }
                    }


                    pictures.insert(MapOfConstAnnotatedPictures::value_type(picture->filename, picture));

                    //ROS_INFO_STREAM("Picture [" << picture->filename << "] inserted. Size: " << picture->completeFile.size());

                    // Calls the listeners to tell them about the new picture
                    BOOST_FOREACH(InFieldPicsManager::Listener* l, listeners)
                    {
                        if (picture->completeFile.size() == 0)
                        {
                            l->onNewInFieldPicTaken(picture);
                        }
                        else
                        {
                            l->onInFieldPicReceived(picture);
                        }
                    }
                }

                assert(getPicture(picture->filename)->completeFile.size() == picture->completeFile.size());

                // Request the complete file (the picture data is not sent by default, to save bandwidth)
                if (picture->completeFile.size() == 0)
                {
                    EXIFReader_msgs::GetAnnotatedPicture::Request req;
                    req.filename = picture->filename;
                    EXIFReader_msgs::GetAnnotatedPicture::Response res;
                    servGetPic.call(req, res); // todo. THIS IS A BLOCKING CALL. Would be better to get a callback
                    onNewInFieldPicMsgReceived(&res.picture);
                }

            }
            
            void InFieldPicsManager::processNewModification(const EXIFReader_msgs::Modification& modificationMsg)
            {
                //ROS_INFO_STREAM("Processing a modification message for [" << modificationMsg.filename << "]");           
                
                AnnotatedPicture* pic = NULL;

                // This is dangerous because we could end up blocking the ROS thread (ROS spin)
                boost::mutex::scoped_lock lock(mutexPictures);
                {
                    try
                    {
                        // This is a hack, but it's the only way in C++ to be able to expose a map of const objects.
                        // If I don't do this, then everybody can modify my objects.
                        pic = const_cast<AnnotatedPicture*> (pictures.at(modificationMsg.filename));
                    }
                    catch (std::out_of_range) // If the filename does not exist
                    {
                        cerr << "Received a modification message for a picture that does not exist: " << modificationMsg.filename << std::endl;
                        return;
                    }

                }

                pic->annotation = modificationMsg.annotation;
                
                // Todo: tell the listeners about the modification
                // Calls the listeners to tell them about the new modification
//                    BOOST_FOREACH(InFieldPicsManager::Listener* l, listeners)
//                    {
//                        l->onPictureModified(picture);
//                    }
            }

        } // End class
    }
}

