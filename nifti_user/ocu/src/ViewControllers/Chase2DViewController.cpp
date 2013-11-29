// Benoit 2011-09-13
// Based on RobotCentricViewController from September 2010

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "IVisualizationManager.h"
#include "NIFTiConstants.h"
#include "Panels/MultiVizKeyEvent.h"
#include "Panels/MultiVizMouseEvent.h"

#include "ViewControllers/Chase2DViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                // Todo: Make the robot not directly in the center, but rather in a 1/3, 2/3 setup
                const Ogre::Vector3 Chase2DViewController::POSITION_INITIAL = Ogre::Vector3(Ogre::Vector3(0, 0, 999)); // Sideways, up, forward. Note: The value is not important, except that it must be above all elements in the map (otherwise these elements won't be shown)

                const int Chase2DViewController::ZOOM_MIN = 60; // Zoomed out high above the robot
                const int Chase2DViewController::ZOOM_MAX = 250; // Zoomed in close to the ground
                const float Chase2DViewController::ZOOM_SPEED = 0.001; // How fast the camera zooms with one move of the mouse

                Chase2DViewController::Chase2DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer)
                : OrthogonalViewController(sceneMgr, vizMgr, camera, frameTransformer, NIFTiConstants::getBaseTFForRobot())
                {
                }
                
                Chase2DViewController::~Chase2DViewController()
                {
                }

//                void Chase2DViewController::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
//                {
//                    printf("%s\n", toString().c_str());
//                }

                void Chase2DViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    bool moved = false;

                    if (evt.evt.GetWheelRotation() != 0)
                    {
                        int diff = evt.evt.GetWheelRotation();
                        scale *= 1.0 - (-diff) * ZOOM_SPEED;

                        if (scale < ZOOM_MIN) scale = ZOOM_MIN;
                        if (scale > ZOOM_MAX) scale = ZOOM_MAX;

                        moved = true;
                    }

                    if (moved)
                    {
                        updateCameraProjectionMatrix();

                        vizMgr->updateOCUInfoViewContent();
                        evt.vizMgr->queueRender();
                    }
                }

                void Chase2DViewController::onUpdate(float dt, float ros_dt)
                {
                    /**
                     * 2011-09-13 I removed this and the camera is still updated 
                     *  properly. However, if I start the GUI with this mode as
                     * the default, then the display is weird, until I move or
                     * zoom.
                     * 
                     */ 
                    updateCameraProjectionMatrix();
                }

                void Chase2DViewController::lookAt(const Ogre::Vector3& point)
                {
                    // Todo: The only thing that can be done here is zooming in or out to see the point
                }

                void Chase2DViewController::resetView()
                {
                    camera->setPosition(POSITION_INITIAL);

                    // Update September 2011: I changed the orientation again, because it was still at 90 deg.
                    // Creates the default orientation (north up, view from top)
                    // These numbers changed after a bug that appeared in FDDO in January 
                    // 2011. For some reason, the map started being displayed turned at 
                    // 90 deg.
                    // Ticket #25
                    Ogre::Quaternion orientation;
//                    orientation.x = 0;
//                    orientation.y = -0.707106781;
//                    orientation.z = 0;
//                    orientation.w = 0.707106781;
//                    orientation.x = 0.5;
//                    orientation.y = 0.5;
//                    orientation.z = 0.5;
//                    orientation.w = -0.5;
                    
                    // These below were used for the first part of 2012 (Ubuntu 10.04, ROS Diamondback)
                    //orientation.x = 0.707106781;
                    //orientation.y = 0;
                    //orientation.z = 0;
                    //orientation.w = -0.707106781;
                    
                    // These below are for October 2012+ (Ubuntu 12.04 and ROS Fuerte)
                    orientation.w = 1;
                    orientation.x = 0;
                    orientation.y = 0;
                    orientation.z = -1;
                 
                    camera->setOrientation(orientation);
                    
                    
                    scale = (ZOOM_MIN + ZOOM_MAX) / 3;
                    updateCameraProjectionMatrix();

                }

            }
        }
    }
}
