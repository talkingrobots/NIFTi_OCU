// Benoit 2011-11-16

#ifndef EU_NIFTI_OCU_NIFTI_VIEWS_UTIL_H
#define EU_NIFTI_OCU_NIFTI_VIEWS_UTIL_H

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Mode of arrangement of the multiple displays
             */
            enum MultiVizMode
            {
                VIZ_MODE_SINGLE, VIZ_MODE_DUAL_VERTICAL, VIZ_MODE_DUAL_HORIZONTAL, VIZ_MODE_QUAD
            };

            /**
             * The types of views and view controllers that can be used in each visualization panel
             * They come from nifti_ocu_msgs::NIFTiGUIStatus.msg
             */
            enum ViewType
            {
                VIEW_TYPE_OMNI_CAM, VIEW_TYPE_VIRTUAL_PTZ_CAM, VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY, VIEW_TYPE_ARM_CAM, VIEW_TYPE_UAV_CAM_DOWN, VIEW_TYPE_UAV_CAM_FRONT, VIEW_TYPE_PTZ_CAM, VIEW_TYPE_KINECT_CAM, VIEW_TYPE_MAP_2D, VIEW_TYPE_MAP_3D, VIEW_TYPE_CHASE_2D, VIEW_TYPE_CHASE_3D, VIEW_TYPE_FIRST_PERSON, VIEW_TYPE_PHOTO, VIEW_TYPE_UNINITIALIZED
            };
            
            /**
             * The categories of views that can be used in each visualization panel
             */
            enum ViewCategory
            {
                VIEW_CATEGORY_CAMERA, VIEW_CATEGORY_VIRTUAL_SCENE, VIEW_CATEGORY_PHOTO, VIEW_CATEGORY_UNINITIALIZED
            };

            class NIFTiViewsUtil
            {
            public:
                
                static ViewCategory getViewCategory(ViewType type);
                
                static bool viewTypeIsValid(ViewType type);
                
                static bool viewTypeIsCamera(ViewType type);

                static bool viewTypeIsVirtualScene(ViewType type);
                
                static bool viewTypeIsPhoto(ViewType type);
                
                static bool viewTypeIsInitialized(ViewType type);
                
                static std::string intToBinaryString(unsigned long n);

                static unsigned int getVisiblityFlag(std::string displayName);

                static unsigned int getVisiblityMask(ViewType type);
            };
        }
    }
}

#endif // EU_NIFTI_OCU_NIFTI_VIEWS_UTIL_H
