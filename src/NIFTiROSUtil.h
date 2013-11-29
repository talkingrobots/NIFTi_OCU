// Benoit 2010-06-11

#include <string>

#ifndef EU_NIFTI_OCU_NIFTI_ROS_UTIL_H
#define EU_NIFTI_OCU_NIFTI_ROS_UTIL_H

namespace ros
{
    class NodeHandle;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Contains a collection of constants that are used throughout the
             * application.
             */
            class NIFTiROSUtil
            {
            public:

                static ros::NodeHandle* getNodeHandleWithPrefix();
                static ros::NodeHandle* getNodeHandle();

                // Gets parameters from the launch file (with namespace "nifti_gui" or "nifti_gui123..." when launched without launch file
                static bool getParam(const std::string& key, std::string& s);
                static bool getParam(const std::string& key, bool& b);

            protected:

                // This node will prefix parameters and topic names with the name of this node, which is unique for every instance
                static ros::NodeHandle* nodeHandleWithPrefix;

                // This node is a standard one that should be used throughout the program
                static ros::NodeHandle* nodeHandle;

            };

        }
    }
}



#endif  // EU_NIFTI_OCU_NIFTI_ROS_UTIL_H
