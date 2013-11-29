// Benoit 2011-05-16

#ifndef EU_NIFTI_OCU_UUID_MANAGER_H
#define EU_NIFTI_OCU_UUID_MANAGER_H

#include <queue>

#include <wx/thread.h>

#include <ros/service_client.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Manages the available UUIDs, so that there are always some available
             */
            class UUIDsManager : public wxThread
            {
            public:

                static UUIDsManager* getInstance();
                void stopManaging();

                static int getUUID();

            private:
                UUIDsManager();

                void* Entry();

                ros::ServiceClient serviceClient;

                std::queue<int> availableUUIDs;
                wxMutex mutexForQueue;
                wxMutex mutexForCondition;
                wxCondition condition;

                static UUIDsManager* instance;
                bool keepManaging;

                static const char* TOPIC;
                static const u_int NUM_REQUESTED;
            };


        }
    }
}



#endif // EU_NIFTI_OCU_UUID_MANAGER_H
