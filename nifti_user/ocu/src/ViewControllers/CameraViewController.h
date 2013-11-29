// Benoit 2011-09-14

#ifndef EU_NIFTI_OCU_VIEW_CAMERA_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_CAMERA_VIEW_CONTROLLER_H

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

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
                 * Control for real physical cameras
                 */
                class CameraViewController : public rviz::ViewController
                {
                public:
                    CameraViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame);
                    virtual ~CameraViewController();
                    
                    virtual Ogre::Vector3 getPosition() const;
                    virtual Ogre::Quaternion getOrientation() const;
                    
                protected:
                    void onActivate();
                    void onDeactivate();
                    void onUpdate(float dt, float ros_dt);

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_CAMERA_VIEW_CONTROLLER_H
