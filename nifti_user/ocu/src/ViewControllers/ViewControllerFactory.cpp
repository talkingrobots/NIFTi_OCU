// Benoit September 2010

#include "NIFTiConstants.h"

#include "ViewControllers/ArmCamViewController.h"
#include "ViewControllers/BlankViewController.h"
#include "ViewControllers/Chase2DViewController.h"
#include "ViewControllers/Chase3DViewController.h"
#include "ViewControllers/FirstPersonViewController.h"
#include "ViewControllers/Map2DViewController.h"
#include "ViewControllers/Map3DViewController.h"
#include "ViewControllers/FixedCamViewController.h"
#include "ViewControllers/PTZCamViewController.h"
#include "ViewControllers/VirtualPTZViewController.h"

#include "ViewControllers/ViewControllerFactory.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                IViewController* ViewControllerFactory::createViewController(ViewType viewType, Ogre::SceneManager* sceneMgr, Ogre::Camera* camera, IVisualizationManager* vizMgr, rviz::FrameTransformer* frameTransformer)
                {
                    switch (viewType)
                    {
                        case VIEW_TYPE_OMNI_CAM:
                            return new FixedCamViewController(sceneMgr, vizMgr, camera, frameTransformer, "/pano", 2 * NIFTiConstants::PI, NIFTiConstants::PI);
                        case VIEW_TYPE_VIRTUAL_PTZ_CAM:
                        case VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY:
                            return new VirtualPTZViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_ARM_CAM:
                            return new ArmCamViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_UAV_CAM_DOWN:
                            return new FixedCamViewController(sceneMgr, vizMgr, camera, frameTransformer, "/uav_cam_down", 1.101, 0.863); // Logitech C905, FoV diag is 75 deg. From my calculations, this gives 63 deg h, 49 deg v
                        case VIEW_TYPE_UAV_CAM_FRONT:
                            return new FixedCamViewController(sceneMgr, vizMgr, camera, frameTransformer, "/uav_cam_front", 1.101, 0.863); // Logitech C905, FoV diag is 75 deg. From my calculations, this gives 63 deg h, 49 deg v
                        case VIEW_TYPE_PTZ_CAM:
                            return new PTZCamViewController(sceneMgr, vizMgr, camera, frameTransformer); // This dates from 2010 and needs to be re-tested
                        case VIEW_TYPE_KINECT_CAM:
                            return new FixedCamViewController(sceneMgr, vizMgr, camera, frameTransformer, "/omni_frame", 0.994837674, 0.750491578); // 57 degrees, 43 degrees found on the web
                        case VIEW_TYPE_FIRST_PERSON:
                            return new rviz::FirstPersonViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_MAP_2D:
                            return new Map2DViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_MAP_3D:
                            return new Map3DViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_CHASE_2D:
                            return new Chase2DViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_CHASE_3D:
                            return new Chase3DViewController(sceneMgr, vizMgr, camera, frameTransformer);
                        case VIEW_TYPE_PHOTO:
                            return NULL; // Do not use the factory for the PhotoViewController. It is too different.
                        case VIEW_TYPE_UNINITIALIZED:
                            return new BlankViewController();
                        default:
                            throw "Invalid ViewType";
                    }
                }

            }
        }
    }
}
