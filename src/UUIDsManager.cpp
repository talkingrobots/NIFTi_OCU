// Benoit 2011-05-16

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/rate.h>

#include <eu_nifti_env_msg_ros/RequestForUUIDs.h>

#include "NIFTiROSUtil.h"

#include "UUIDsManager.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const char* UUIDsManager::TOPIC = "/eoi/RequestForUUIDs";
            const u_int UUIDsManager::NUM_REQUESTED(10);

            UUIDsManager* UUIDsManager::instance = NULL;

            UUIDsManager* UUIDsManager::getInstance()
            {
                if (instance == NULL)
                {
                    instance = new UUIDsManager();
                }
                return instance;
            }

            UUIDsManager::UUIDsManager() :
            wxThread(wxTHREAD_JOINABLE)
            , condition(mutexForCondition)
            , keepManaging(true)
            {

            }

            void UUIDsManager::stopManaging()
            {
                //printf("IN UUIDsManager::stopManaging \n");
                
                keepManaging = false;

                if (instance != NULL)
                {
                    instance->condition.Signal(); // Will make it go out of the main loop and terminate
                }
                
                //printf("OUT UUIDsManager::stopManaging \n");
            }

            void* UUIDsManager::Entry()
            {
                //printf("%s\n", "UUIDsManager::Entry()");

                ::ros::ServiceClient client = NIFTiROSUtil::getNodeHandle()->serviceClient<eu_nifti_env_msg_ros::RequestForUUIDs > (TOPIC);
                eu_nifti_env_msg_ros::RequestForUUIDs srv;
                srv.request.numRequested = NUM_REQUESTED;

                mutexForCondition.Lock(); // This must be called before the first wait()


                while (keepManaging)
                {
                    //ROS_INFO("In the loop (keepManaging)");
                    
                    // This is a requirement from wxThread
                    // http://docs.wxwidgets.org/2.8/wx_wxthread.html#wxthreadtestdestroy
                    if (TestDestroy())
                        break;
                    
                    if (!client.call(srv))
                    {
                        std::cerr << "Failed to call ROS service \"RequestForUUIDs\"" << std::endl;
                        //return 0; // Removed this on 2012-03-02 so that it would re-check the service every time, since CAST is unstable

                    }
                    else
                    {
                        //ROS_INFO("Got UUIDs");
                        {
                            wxMutexLocker lock(instance->mutexForQueue);

                            // Adds all new UUIDs to the list
                            for (u_int i = 0; i < srv.response.uuids.size(); i++)
                            {
                                availableUUIDs.push(srv.response.uuids.at(i));
                                //std::cout << "Added " <<  srv.response.uuids.at(i) << std::endl;
                            }
                        }
                    }


                    // Waits until more ids are needed (signal will be called on the condition)
                    //ROS_INFO("Before waiting");
                    condition.Wait();
                    //ROS_INFO("After waiting");

                }

                return 0;
            }

            int UUIDsManager::getUUID()
            {
                //ROS_INFO("int UUIDsManager::getUUID()");

                int uuid;

                {
                    wxMutexLocker lock(instance->mutexForQueue);

                    //ROS_INFO("Num left: %i/%i", instance->availableUUIDs.size(), NUM_REQUESTED);
                    //ROS_INFO("Enough? %i", instance->availableUUIDs.size() <= NUM_REQUESTED / 2);

                    assert(instance != NULL);
                    
                    // Requests more id's when the list is half empty
                    if (instance->availableUUIDs.size() <= NUM_REQUESTED / 2)
                    {
                        //ROS_INFO("Will try waking up the thread to request UUIDs");
                        instance->condition.Signal();

                        if (instance->availableUUIDs.size() == 0)
                        {
                            throw "No UUID available";
                        }
                    }

                    uuid = instance->availableUUIDs.front();
                    instance->availableUUIDs.pop();
                }

                //ROS_INFO("int UUIDsManager::getUUID() returned %i. Num left: %i", uuid, instance->availableUUIDs.size());

                return uuid;
            }


        }
    }
}

