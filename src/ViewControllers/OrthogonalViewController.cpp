// Benoit 2011-09-13

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include <ogre_tools/orthographic.h>

#include <stdint.h>
#include <sstream>

#include "IVisualizationManager.h"

#include "ViewControllers/OrthogonalViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                OrthogonalViewController::OrthogonalViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame)
                : ViewController(sceneMgr, vizMgr, camera, frameTransformer, referenceFrame)
                {
                }
                
                OrthogonalViewController::~OrthogonalViewController()
                {
                }
                
                double OrthogonalViewController::getFieldOfViewHorizontal() const
                {
                    return 0; // The field of view does not make sense here because it is not a perspective view but rather an orthogonal view
                }
                
                double OrthogonalViewController::getFieldOfViewVertical() const
                {
                    return 0; // The field of view does not make sense here because it is not a perspective view but rather an orthogonal view
                }

                void OrthogonalViewController::onActivate()
                {
                    assert(camera != NULL);
                    assert(referenceNode != NULL);
                    
                    // Sets-up the camera
                    camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
                    resetView();

                    try
                    {
                        referenceNode->attachObject(camera);
                    }
                    catch(Ogre::Exception e)
                    {
                        printf("%s\n", e.getFullDescription().c_str());
                        assert(false);
                    }
                }

                void OrthogonalViewController::onDeactivate()
                {
                    referenceNode->detachObject(camera);
                    camera->setCustomProjectionMatrix(false);
                }

                void OrthogonalViewController::updateCameraProjectionMatrix()
                {
                    float width = camera->getViewport()->getActualWidth();
                    float height = camera->getViewport()->getActualHeight();

                    Ogre::Matrix4 proj;
                    ogre_tools::buildScaledOrthoMatrix(proj, -width / scale / 2, width / scale / 2, -height / scale / 2, height / scale / 2, camera->getNearClipDistance(), camera->getFarClipDistance());
                    camera->setCustomProjectionMatrix(true, proj);
                    
                    vizMgr->updateOCUInfoViewContent();
                }

                std::string OrthogonalViewController::toString() const
                {
                    std::ostringstream oss;
                    oss << scale << " " << camera->getPosition().x << " " << camera->getPosition().y << " " << camera->getPosition().z
                            << " " << camera->getOrientation().x << " " << camera->getOrientation().y << " " << camera->getOrientation().z << " " << camera->getOrientation().w;

                    return oss.str();
                }


            }
        }
    }
}
