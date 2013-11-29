// Benoit 2011-09-08

#ifndef EU_NIFTI_OCU_VIEW_CHASE_3D_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_CHASE_3D_VIEW_CONTROLLER_H

#include "ViewControllers/PerspectiveViewController.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace eu 
{
    namespace nifti 
    {
        namespace ocu 
        {
            namespace view 
            {

                /**
                 * Places the camera above and behind the robot, and allows the
                 * user to zoom in our out making the camera move along a 
                 * "banana" and to turn the camera like an orbit around the robot
                 */
                class Chase3DViewController : public PerspectiveViewController 
                {
                public:
                    Chase3DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~Chase3DViewController();

                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    std::string toString() const;

                    void lookAt(const Ogre::Vector3& point);
                    void resetView(); 

                protected:
                    
                    void moveAlongBanana(float angle);

                    void normalizeAngleOnBanana();
                    void normalizeAngleOnHorizontalPlane();

                    void updateCamera();

                    float angleBanana; // 0 is at the top, pi/2 is at the robot
                    float angleOnHorizontalPlane; // Angle of the camera position as viewed from the top


                    static const float VERTICAL_OFFSET; // The banana is this much above the ground
                    
                    static const float ANGLE_ON_HORIZONTAL_PLANE_INITIAL;
                    
                    static const float RADIUS_BANANA;
                    
                    static const float ANGLE_ON_BANANA_INITIAL;
                    
                    static const float ANGLE_ON_BANANA_MIN;
                    static const float ANGLE_ON_BANANA_MAX;

                    static const float ZOOM_SPEED_DRAG;

                    static const float TURN_SPEED;

                    static const float ZOOM_SPEED_MOUSE_WHEEL;

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_CHASE_3D_VIEW_CONTROLLER_H
