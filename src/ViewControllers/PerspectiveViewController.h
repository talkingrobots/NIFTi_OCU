// Benoit 2011-09-14

#ifndef EU_NIFTI_OCU_VIEW_PERSPECTIVE_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_PERSPECTIVE_VIEW_CONTROLLER_H

#include "ViewControllers/ViewController.h"

namespace ogre_tools 
{
    class Shape;
    class SceneNode;
}

namespace eu 
{
    namespace nifti 
    {
        namespace ocu 
        {
            namespace view 
            {

                /**
                 * Camera with a standard 3D perspective
                 */
                class PerspectiveViewController : public rviz::ViewController 
                {
                public:
                    PerspectiveViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame);
                    virtual ~PerspectiveViewController();
                    
                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;

                    virtual void resetView() = 0;
                    
                    virtual std::string toString() const;

                protected:
                    void onActivate();
                    void onDeactivate();
                    
                    void onUpdate(float dt, float ros_dt);

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_PERSPECTIVE_VIEW_CONTROLLER_H
