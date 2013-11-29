// Benoit 2011-02-23

#include <vector>
#include <wx-2.8/wx/thread.h>

#include "OCUMessagePublisherThread.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const char* OCUMessagePublisherThread::TOPIC = "/ocu/info";
            const int OCUMessagePublisherThread::FREQUENCY_IN_HZ(1);
            
            const int OCUMessagePublisherThread::MAX_NUM_VIEWS = 4;

            OCUMessagePublisherThread::OCUMessagePublisherThread() :
            wxThread(wxTHREAD_JOINABLE)
            , keepPublishing(true)
            {
                // Initializes the arrays (I don't know how to do it in a better way)
                this->guiStatus.views.resize(MAX_NUM_VIEWS);
                this->guiStatus.viewParams.resize(MAX_NUM_VIEWS);

                publisher = rosNodeHandle.advertise<nifti_ocu_msgs::NIFTiGUIStatus > (TOPIC, 1);
            }

            void OCUMessagePublisherThread::stopPublishing()
            {
                //printf("%s\n", "OCUMessagePublisherThread::stopPublishing");
                keepPublishing = false;
            }

            void* OCUMessagePublisherThread::Entry()
            {

                //printf("%s\n", "OCUMessagePublisherThread::Entry()");

                ros::Rate pubRate(FREQUENCY_IN_HZ);

                while (keepPublishing)
                {
                    // This is a requirement from wxThread
                    // http://docs.wxwidgets.org/2.8/wx_wxthread.html#wxthreadtestdestroy
                    if (TestDestroy())
                        break;

                    guiStatus.header.stamp = ros::Time::now();
                    publisher.publish(guiStatus);
                    //printf("Published OCU info\n");

                    pubRate.sleep();
                }

                return 0;
            }


        }
    }
}

