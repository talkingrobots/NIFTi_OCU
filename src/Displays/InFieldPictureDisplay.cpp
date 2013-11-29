// Benoit 2013-01-17

#include <iostream>

#include <boost/foreach.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "FloatValidator.h"
#include "InFieldPicsManager.h"

#include "Selection/SelectionManager.h"

#include "Displays/Markers/InFieldPictureMarker.h"

#include "Displays/InFieldPictureDisplay.h"

using namespace std;
using namespace eu::nifti::ocu;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                InFieldPictureDisplay::InFieldPictureDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, eu::nifti::ocu::selection::SelectionManager* selMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
                : rviz::Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
                , selMgr(selMgr)
                {
                    scene_node_ = sceneMgr->getRootSceneNode()->createChildSceneNode();
                }

                InFieldPictureDisplay::~InFieldPictureDisplay()
                {
                    unsubscribe();

                    clearMarkers();
                }

                void InFieldPictureDisplay::onSelectionManagerDeleted()
                {
                    selMgr = NULL;
                }

                void InFieldPictureDisplay::onEnable()
                {
                    subscribe();

                    scene_node_->setVisible(true);
                }

                void InFieldPictureDisplay::onDisable()
                {
                    unsubscribe();

                    scene_node_->setVisible(false);
                }

                void InFieldPictureDisplay::subscribe()
                {
                    // That was from RVIZ, but I want to stay subscribed at all times
                    //if (!isEnabled())
                    //{
                    //    return;
                    //}

                    InFieldPicsManager::instance()->addListener(this);
                }

                void InFieldPictureDisplay::unsubscribe()
                {
                    //std::cout << "IN void InFieldPictureDisplay::unsubscribe()" << std::endl;

                    InFieldPicsManager::instance()->removeListener(this);
                }

                void InFieldPictureDisplay::update(float wall_dt, float ros_dt)
                {
                    std::vector<const EXIFReader_msgs::AnnotatedPicture*> localQueue;

                    // Swaps the main queue with a local, to process the messages while still being able to receive some
                    {
                        boost::mutex::scoped_lock lock(pictureQueueMutex);

                        localQueue.swap(pictureQueue);
                    }

                    // If messages have been received since the last update
                    if (!localQueue.empty())
                    {
                        std::vector<const EXIFReader_msgs::AnnotatedPicture*>::const_iterator message_it = localQueue.begin();
                        std::vector<const EXIFReader_msgs::AnnotatedPicture*>::const_iterator message_end = localQueue.end();
                        for (; message_it != message_end; ++message_it)
                        {
                            addMarker(*message_it);
                        }
                    }
                }

                void InFieldPictureDisplay::onInFieldPicReceived(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //cout << "InFieldPictureDisplay::onNewInFieldPicReceived" << endl;

                    boost::mutex::scoped_lock lock(pictureQueueMutex);

                    // Stores the picture in an ordered list to retrieve it easily when the user clicks on the thumbnail
                    pictureQueue.push_back(picture);
                }

                void InFieldPictureDisplay::onNewInFieldPicTaken(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //cout << "InFieldPictureDisplay::onNewInFieldPicTaken" << endl;
                    // For now, do nothing. Wait until the full picture comes
                }

                void InFieldPictureDisplay::addMarker(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //std::cout << "IN void InFieldPictureDisplay::addMarker at ROS time " << msg->header.stamp << std::endl;

                    // That means that this picture was taken from a mobile device but did not contain GPS coordinates
                    // Or some value is invalid
                    if ((picture->pose.position.x == 0 && picture->pose.position.y == 0 && picture->pose.position.z == 0)
                        || !rviz::FloatValidator::validateFloats(picture->pose))
                    {
                        return;
                    }

                    InFieldPictureMarker* marker = new InFieldPictureMarker(sceneMgr, scene_node_, picture);

                    // Stores the marker and ensures that it did not already exist
                    std::pair < MapOfInFieldPictureMarkers::const_iterator, bool> alreadyExistsCheck = markers.insert(MapOfInFieldPictureMarkers::value_type(picture->filename, marker));
                    assert(alreadyExistsCheck.second == true);

                    marker->addToOgre();

                    // Tells the Selection Manager to keep track of this object
                    assert(selMgr != NULL);
                    rviz::CollObjectHandle handle = selMgr->addTrackedMarker(marker);
                    marker->setCollisionObjectHandle(handle);

                    causeRender();

                    //std::cout << "OUT void InFieldPictureDisplay::addMarker at ROS time " << msg->header.stamp << std::endl;
                }

                void InFieldPictureDisplay::clearMarkers()
                {

                    BOOST_FOREACH(const MapOfInFieldPictureMarkers::value_type& pair, markers)
                    {
                        // Goes through all markers and notifies the Selection Manager not to follow these objects anymore
                        if (selMgr != NULL)
                        {
                            selMgr->removeTrackedMarker(pair.second->getCollisionObjectHandle());
                        }
                        delete pair.second;
                    }
                    markers.clear();
                }

                void InFieldPictureDisplay::fixedFrameChanged()
                {
                    // Assuming that /map will always be the reference frame
                }

                void InFieldPictureDisplay::reset()
                {
                    rviz::Display::reset();
                    clearMarkers();
                }

            }
        }
    }
}
