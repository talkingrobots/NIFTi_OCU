// Benoit 2013-01-11

#ifndef EU_NIFTI_OCU_IN_FIELD_PICS_MANAGER_H
#define EU_NIFTI_OCU_IN_FIELD_PICS_MANAGER_H

#include <map>
#include <queue>
#include <set>
#include <string>

#include <boost/thread.hpp>

#include <ros/subscriber.h>
#include <ros/service_client.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>
#include <EXIFReader_msgs/Modification.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            typedef std::map< const std::string, const EXIFReader_msgs::AnnotatedPicture*> MapOfConstAnnotatedPictures;

            /**
             * Listens on ROS and manages the pictures coming from mobile devices.
             */
            class InFieldPicsManager
            {
            public:

                /**
                 * Such a listener gets notified when a new picture is received from ROS
                 * @param picture
                 */
                class Listener
                {
                public:
                    // Called when a complete picture is received
                    virtual void onInFieldPicReceived(const EXIFReader_msgs::AnnotatedPicture* picture) = 0;

                    // Called when a notification of a picture is received (without the complete file)
                    virtual void onNewInFieldPicTaken(const EXIFReader_msgs::AnnotatedPicture* picture) = 0;
                };

                static InFieldPicsManager* instance();

                void init();
                void unInit();

                std::list<const EXIFReader_msgs::AnnotatedPicture*>* addListener(InFieldPicsManager::Listener* listener, bool returnExistingPictures = false);
                void removeListener(InFieldPicsManager::Listener* listener);

                std::list<const EXIFReader_msgs::AnnotatedPicture*>* getPictures();

                const EXIFReader_msgs::AnnotatedPicture* getPicture(std::string filename);

            protected:

                void onNewInFieldPicMsgReceived(const EXIFReader_msgs::AnnotatedPictureConstPtr& picture);
                void onNewInFieldPicMsgReceived(const EXIFReader_msgs::AnnotatedPicture* picture);
                void onModificationReceived(const EXIFReader_msgs::Modification& modificationMsg);

                void processNewMessages();
                void processNewPicture(const EXIFReader_msgs::AnnotatedPicture* picture);
                void processNewModification(const EXIFReader_msgs::Modification& modificationMsg);

                // Thread that will call the listeners upon reception of pictures from the server
                boost::thread threadToCallListeners;

                boost::condition_variable conditionNewROSMessages;
                boost::mutex mutexNewROSMessages;
                boost::mutex mutexPictures;

                std::set<InFieldPicsManager::Listener*> listeners;
                boost::mutex mutexListeners;

                ros::Subscriber subscriberNewPics;
                ros::ServiceClient servGetPics;
                ros::ServiceClient servGetPic;
                ros::Subscriber subscriberModification;

                MapOfConstAnnotatedPictures pictures;
                std::queue<const EXIFReader_msgs::AnnotatedPicture*> newPictures;
                std::queue<EXIFReader_msgs::Modification> newModifications;

                bool shutdownRequested;

            private:
                InFieldPicsManager(); // This class is a singleton
                static InFieldPicsManager* theInstance;
            };



        }
    }
}



#endif
