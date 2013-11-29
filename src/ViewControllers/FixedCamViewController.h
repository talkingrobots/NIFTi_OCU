// Benoit 2013-05-03

#ifndef EU_NIFTI_OCU_VIEW_FIXED_CAM_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_FIXED_CAM_VIEW_CONTROLLER_H

#include "ViewControllers/CameraViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                /**
                 * This controller does nothing (the view is fixed)
                 */
                class FixedCamViewController : public CameraViewController
                {
                public:
                    FixedCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame, double fieldOfViewHorizontal, double fieldOfViewVertical);
                    virtual ~FixedCamViewController();

                    std::string toString() const;

                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;
                    
                    void lookAt(const Ogre::Vector3& point);
                    void resetView();

                protected:
                    
                    double fieldOfViewHorizontal, fieldOfViewVertical;
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_FIXED_CAM_VIEW_CONTROLLER_H
