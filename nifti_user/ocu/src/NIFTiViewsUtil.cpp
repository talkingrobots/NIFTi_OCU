// Benoit 2011-11-16

#include <string>

#include "NIFTiViewsUtil.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            ViewCategory NIFTiViewsUtil::getViewCategory(ViewType type)
            {
                if(type <= VIEW_TYPE_KINECT_CAM) return VIEW_CATEGORY_CAMERA;
                if(type <= VIEW_TYPE_FIRST_PERSON) return VIEW_CATEGORY_VIRTUAL_SCENE;
                if(type <= VIEW_TYPE_PHOTO) return VIEW_CATEGORY_PHOTO;
                return VIEW_CATEGORY_UNINITIALIZED;
            }
            
            bool NIFTiViewsUtil::viewTypeIsValid(ViewType type)
            {
                return type >= VIEW_TYPE_OMNI_CAM && type <= VIEW_TYPE_UNINITIALIZED;
            }
            
            bool NIFTiViewsUtil::viewTypeIsCamera(ViewType type)
            {
                return type <= VIEW_TYPE_KINECT_CAM;
            }

            bool NIFTiViewsUtil::viewTypeIsVirtualScene(ViewType type)
            {
                return type >= VIEW_TYPE_MAP_2D && type <= VIEW_TYPE_FIRST_PERSON;
            }
            
            bool NIFTiViewsUtil::viewTypeIsPhoto(ViewType type)
            {
                return type == VIEW_TYPE_PHOTO;
            }

            bool NIFTiViewsUtil::viewTypeIsInitialized(ViewType type)
            {
                return type != VIEW_TYPE_UNINITIALIZED;
            }

            // Taken from http://www.cplusplus.com/forum/general/10898/
            std::string NIFTiViewsUtil::intToBinaryString(unsigned long n)
            {
                char result[ (sizeof ( unsigned long) * 8) + 1 ];
                unsigned index = sizeof ( unsigned long) * 8;
                result[ index ] = '\0';

                do result[ --index ] = '0' + (n & 1);
                while (n >>= 1);

                return std::string(result + index);
            }

            unsigned int NIFTiViewsUtil::getVisiblityFlag(std::string displayName)
            {
                // The distance circles around the robot have bit 1 (00000001)
                // The laser scan has bit 2 (00000010)
                // The map has bit 3 (00000100)
                // The point cloud(s) has bit 4 (00001000)

                if (displayName == "Distance")
                {
                    return 0x00000001;
                }
                else if (displayName == "Map2D")
                {
                    return 0x00000004;
                }
                else if (displayName == "Horizontal Laser Scan")
                {
                    return 0x00000002;
                }
                else if (displayName == "Point Cloud 2 - Map" || "Point Cloud 2 - Dynamic" || "Point Cloud UAV")
                {
                    return 0x00000008;
                }

                return 0x11111111;
            }

            unsigned int NIFTiViewsUtil::getVisiblityMask(ViewType type)
            {
                // The Map View wants to hide the distance, laser, and point clouds (00000100) (F4)
                // The Chase 2D View wants to hide the point clouds (00000111) (F7)
                // The Map 3D View wants to hide the distance, and laser (00001100) (FC)
                // The Virtual PTZ wants to hide the map (00001011) (FB)

                switch (type)
                {

                    case VIEW_TYPE_CHASE_2D:
                        return 0xFFFFFFF7;
                    case VIEW_TYPE_MAP_3D:
                        return 0xFFFFFFFC;
                    case VIEW_TYPE_MAP_2D:
                        return 0xFFFFFFF4;
                    case VIEW_TYPE_VIRTUAL_PTZ_CAM:
                    case VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY:
                        return 0xFFFFFFFB;
                    default:
                        return 0xFFFFFFFF;
                }

            }

        }
    }
}
