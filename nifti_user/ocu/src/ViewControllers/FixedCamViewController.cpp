// Benoit 2013-05-03

#include "FixedCamViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                FixedCamViewController::FixedCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame, double fieldOfViewHorizontal, double fieldOfViewVertical)
                : CameraViewController(sceneMgr, vizMgr, camera, frameTransformer, referenceFrame)
                , fieldOfViewHorizontal(fieldOfViewHorizontal)
                , fieldOfViewVertical(fieldOfViewVertical)
                {

                }

                FixedCamViewController::~FixedCamViewController()
                {
                }

                std::string FixedCamViewController::toString() const
                {
                    std::stringstream ss;
                    ss << "FixedCamViewController on reference frame " << referenceFrame << " with FoVx " << fieldOfViewHorizontal << " and FoVy " << fieldOfViewVertical;
                    return ss.str();
                }

                double FixedCamViewController::getFieldOfViewHorizontal() const
                {
                    return fieldOfViewHorizontal;
                }

                double FixedCamViewController::getFieldOfViewVertical() const
                {
                    return fieldOfViewVertical;
                }

                void FixedCamViewController::lookAt(const Ogre::Vector3& point)
                {
                }

                void FixedCamViewController::resetView()
                {
                }

            }
        }
    }
}
