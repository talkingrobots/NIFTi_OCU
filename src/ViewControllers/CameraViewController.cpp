// Benoit 2011-09-14

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "FloatValidator.h"
#include "NIFTiROSOgreUtil.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "ViewControllers/CameraViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                CameraViewController::CameraViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame)
                : ViewController(sceneMgr, vizMgr, camera, frameTransformer, referenceFrame)
                {
                }

                CameraViewController::~CameraViewController()
                {

                }

                void CameraViewController::onActivate()
                {
                    resetView();
                }

                void CameraViewController::onDeactivate()
                {
                }

                void CameraViewController::onUpdate(float dt, float ros_dt)
                {
                }

                Ogre::Vector3 CameraViewController::getPosition() const
                {
                    // 1) This is the normal case
                    if (referenceFrame == "/map")
                        return ViewController::getPosition();

                    //std::cout << "Will ask for transform from /map to " << referenceFrame << " use that as the position and orientation" << std::endl;

                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;

                    bool success = rviz::FrameTransformer::transform("/map", referenceFrame, position, orientation);

                    if (success == false || !rviz::FloatValidator::validateFloats(position) || !rviz::FloatValidator::validateFloats(orientation))
                    {
                        std::stringstream ss;
                        ss << "Error transforming TF from /map to " << referenceFrame;
                        throw ss.str();
                    }

                    return position;
                }

                Ogre::Quaternion CameraViewController::getOrientation() const
                {
                    // 1) This is the normal case
                    if (referenceFrame == "/map")
                        return ViewController::getOrientation();

                    // 2)
                    // If the referenceFrame is something else, it is usually because the CameraInfo received is not 
                    // calibrated, or there was none received

                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;

                    bool success = rviz::FrameTransformer::transform("/map", referenceFrame, position, orientation);

                    if (success == false)
                    {
                        std::stringstream ss;
                        ss << "Error transforming TF from /map to " << referenceFrame;
                        throw ss.str();
                    }

                    // Here, we got the orientation wrt TFs, which is not the same orientation
                    NIFTiROSOgreUtil::convertOrientationFromROSToOgre(orientation);

                    return orientation;
                }


            } // End-class
        }
    }
}
