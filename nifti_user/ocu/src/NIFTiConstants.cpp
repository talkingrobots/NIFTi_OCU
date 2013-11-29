// Benoit 2010-06-11

#include <ros/package.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include "NIFTiROSUtil.h"

#include "NIFTiConstants.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            const std::string& NIFTiConstants::getRobotType()
            {
                if (robotType == "")
                {
                    // Ensures that a valid robot type has been defined
                    bool found;
                    found = NIFTiROSUtil::getParam("robotType", robotType);
                    if (!found || (robotType != "UGV" && robotType != "UAV" && robotType != "P3AT"))
                    {
                        robotType = "";
                        throw eu::nifti::misc::ExceptionWithString("Unknown robot type '" + robotType + "'. Valid types are: UGV, UAV, and P3AT.");
                    }
                }
                return robotType;
            }

            const std::string& NIFTiConstants::getBaseTFForRobot()
            {
                if (baseTFForRobot == "")
                {
                    if (getRobotType() == "UGV")
                    {
                        baseTFForRobot = "base_link";
                    }
                    else if (getRobotType() == "UAV")
                    {
                        baseTFForRobot = "uav_base_link";
                    }
                    else if (getRobotType() == "P3AT")
                    {
                        baseTFForRobot = "base_link";
                    }
                    else
                    {
                        // Cannot reach here
                        assert(false);
                    }
                }

                return baseTFForRobot;
            }

            const Ogre::ColourValue NIFTiConstants::DEFAULT_BACKGROUND_COLOR_IN_OGRE_SCENE = Ogre::ColourValue(0.25f, 0.25f, 0.25f, 1.0f); // This color was chosen to be the same as MapDisplay

            const string NIFTiConstants::ROS_PACKAGE_PATH = ros::package::getPath(ROS_PACKAGE_NAME);

            const string NIFTiConstants::SNAPSHOTS_FOLDER = "/snapshots";

            const string NIFTiConstants::FIXED_FRAME_STRING = "<Fixed Frame>";

            const string NIFTiConstants::IMAGE_FOLDER_PATH = "/media/images";

            string NIFTiConstants::robotType = ""; // Will be initialized the first time that it is requested

            string NIFTiConstants::baseTFForRobot = ""; // Will be initialized the first time that it is requested

        }
    }
}

