// Benoit 2011-09-14

#include <sstream>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>

#include "IVisualizationManager.h"

#include "ViewControllers/PerspectiveViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

             
                PerspectiveViewController::PerspectiveViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer, const std::string& referenceFrame)
                : ViewController(sceneMgr, vizMgr, camera, frameTransformer, referenceFrame)
                {
                }
                
                PerspectiveViewController::~PerspectiveViewController()
                {
                }

                double PerspectiveViewController::getFieldOfViewHorizontal() const
                {
                    return vizMgr->getFieldOfViewHorizontal();
                }
                
                double PerspectiveViewController::getFieldOfViewVertical() const
                {
                    return vizMgr->getFieldOfViewVertical();
                }
                
                void PerspectiveViewController::onActivate()
                {
                    camera->setProjectionType(Ogre::PT_PERSPECTIVE); // Ogre default
                    camera->setCustomProjectionMatrix(false); // Ogre default
                    camera->setFOVy(Ogre::Radian(Ogre::Math::PI/4)); // Ogre default

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

                void PerspectiveViewController::onDeactivate()
                {
                    referenceNode->detachObject(camera);
                }
                
                void PerspectiveViewController::onUpdate(float dt, float ros_dt)
                {
                    
                }

                std::string PerspectiveViewController::toString() const
                {
                    using namespace std;

                    std::ostringstream oss;
                    oss << "Position: " << camera->getPosition().x << ", " << camera->getPosition().y << ", " << camera->getPosition().z << endl;
                    oss << "Orientation: " << camera->getOrientation().x << ", " << camera->getOrientation().y << ", " << camera->getOrientation().z << ", " << camera->getOrientation().w << endl;

                    return oss.str();
                }


            }
        }
    }
}