// Benoit September 2010

#ifndef NIFTI_MAP_2D_VIEW_CONTROLLER_H
#define NIFTI_MAP_2D_VIEW_CONTROLLER_H

#include "ViewControllers/OrthogonalViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                class Map2DViewController : public OrthogonalViewController
                {
                public:
                    Map2DViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~Map2DViewController();

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

#endif // NIFTI_MAP_2D_VIEW_CONTROLLER_H
