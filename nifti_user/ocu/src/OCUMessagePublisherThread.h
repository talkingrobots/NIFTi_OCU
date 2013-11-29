// Benoit 2011-02-23

#ifndef OCU_MSG_PUB_THREAD_H
#define OCU_MSG_PUB_THREAD_H

#include <wx/thread.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/rate.h>

#include "nifti_ocu_msgs/NIFTiGUIStatus.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

    class VisualizationFrame;
    
    /**
     * Publishes a message every one second containing info about the OCU. Other
     * objects must populate the message sent, which is stored in this class.
     * @param vizFrame
     */
    class OCUMessagePublisherThread : public wxThread
    {
    public:

        OCUMessagePublisherThread();
        
        void stopPublishing();
        
        nifti_ocu_msgs::NIFTiGUIStatus guiStatus;

    private:
        void* Entry();
        
        ros::NodeHandle rosNodeHandle;
        ros::Publisher publisher;
        
        bool keepPublishing;
        
        static const char* TOPIC;
        static const int FREQUENCY_IN_HZ;
        
        // Maximum number of views that the user can have on the screen at once
        static const int MAX_NUM_VIEWS;
    };


        } }}



#endif // OCU_MSG_PUB_THREAD_H
