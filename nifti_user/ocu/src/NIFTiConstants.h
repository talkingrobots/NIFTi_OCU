// Benoit 2010-06-11

#include <string>

#include <OGRE/OgreColourValue.h>

#ifndef NIFTI_CONSTANTS_H
#define NIFTI_CONSTANTS_H

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
            class NIFTiConstants
            {
            public:
                
                // Returns "UGV", "UAV", or "P3AT". Read from a ROS parameter
                static const std::string& getRobotType();
                
                // Returns "base_link" or "uav_base_link"
                static const std::string& getBaseTFForRobot();

                // Path of the ROS package
                static const std::string ROS_PACKAGE_PATH;
                
                // Sub-folder where the snapshots are stored
                static const std::string SNAPSHOTS_FOLDER;

                // Default string for a Fixed Frame in RVIZ
                static const std::string FIXED_FRAME_STRING; // #define FIXED_FRAME_STRING "<Fixed Frame>"
                
                // Color of the background (sky) in the virtual scene
                static const Ogre::ColourValue DEFAULT_BACKGROUND_COLOR_IN_OGRE_SCENE;
                
                // Path of the main images folder
                static const std::string IMAGE_FOLDER_PATH;
                
                // Minimum button size for touch screen (Recommended from Android Design Guidelines)
                static const short MIN_BUTTON_SIZE = 48;
                
                // Math stuff
                static const double PI = 3.14159265358979323846;
                static const double PI_OVER_2 = 1.57079632679489661923;
                static const double PI_OVER_4 = 0.78539816339744830962;
                static const double DEG_TO_RAD = 2 * 3.14159265358979323846 / 360;

            private:
                
                static std::string robotType;
                static std::string baseTFForRobot;
            };

        }
    }
}



#endif
