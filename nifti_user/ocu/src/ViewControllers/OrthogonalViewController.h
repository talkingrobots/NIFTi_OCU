// Benoit 2011-09-13

#ifndef EU_NIFTI_OCU_VIEW_ORTHOGONAL_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_ORTHOGONAL_VIEW_CONTROLLER_H

#include "ViewControllers/ViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                /**
                 * Camera with a standard 2D map perspective
                 */
                class OrthogonalViewController : public rviz::ViewController
                {
                public:
                    OrthogonalViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame);
                    virtual ~OrthogonalViewController();
                    
                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;

                    virtual std::string toString() const;

                protected:
                    void onActivate();
                    void onDeactivate();
                    
                    void updateCameraProjectionMatrix();

                    float scale;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_ORTHOGONAL_VIEW_CONTROLLER_H
