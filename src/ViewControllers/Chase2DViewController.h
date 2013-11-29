// Benoit 2011-09-13
// Based on RobotCentricViewController from September 2010

#ifndef EU_NIFTI_OCU_VIEW_CHASE_2D_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_CHASE_2D_VIEW_CONTROLLER_H

#include "ViewControllers/OrthogonalViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                class Chase2DViewController : public OrthogonalViewController
                {
                public:
                    Chase2DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~Chase2DViewController();

//                    void handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt);
                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    void lookAt(const Ogre::Vector3& point);
                    void resetView();

                protected:
                    void onUpdate(float dt, float ros_dt);

                    static const Ogre::Vector3 POSITION_INITIAL;
                    
                    static const int ZOOM_MIN;
                    static const int ZOOM_MAX;
                    static const float ZOOM_SPEED;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_CHASE_2D_VIEW_CONTROLLER_H
