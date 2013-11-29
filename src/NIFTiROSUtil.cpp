// Benoit 2010-06-11

//#include <ros/package.h>
#include <ros/this_node.h>
#include <ros/node_handle.h>

#include "NIFTiConstants.h"

#include "NIFTiROSUtil.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            ros::NodeHandle* NIFTiROSUtil::nodeHandleWithPrefix = NULL;
            ros::NodeHandle* NIFTiROSUtil::nodeHandle = NULL;

            ros::NodeHandle* NIFTiROSUtil::getNodeHandleWithPrefix()
            {
                if (nodeHandleWithPrefix == NULL)
                {
                    nodeHandleWithPrefix = new ros::NodeHandle(ros::this_node::getName());
                }
                return nodeHandleWithPrefix;
            }

            ros::NodeHandle* NIFTiROSUtil::getNodeHandle()
            {
                if (nodeHandle == NULL)
                {
                    nodeHandle = new ros::NodeHandle();
                }
                return nodeHandle;
            }

            bool NIFTiROSUtil::getParam(const string& key, string& s)
            {
                string fullKey;
                fullKey.append(ROS_PACKAGE_NAME).append("/").append(key);

                bool found = NIFTiROSUtil::getNodeHandleWithPrefix()->getParam(fullKey, s);

                if (found) return true;

                fullKey = ros::this_node::getName() + "/" + key;

                return NIFTiROSUtil::getNodeHandleWithPrefix()->getParam(fullKey, s);
            }

            bool NIFTiROSUtil::getParam(const string& key, bool& b)
            {
                string fullKey;
                fullKey.append(ROS_PACKAGE_NAME).append("/").append(key);

                //std::cout << "Will search for param " << fullKey << std::endl;

                bool found = NIFTiROSUtil::getNodeHandleWithPrefix()->getParam(fullKey, b);

                if (found) return true;

                fullKey = ros::this_node::getName() + "/" + key;

                return NIFTiROSUtil::getNodeHandleWithPrefix()->getParam(fullKey, b);
            }

        }
    }
}

