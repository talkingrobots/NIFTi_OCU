// Benoit 2010-11-17 Based on the RVIZ MeshResourceMarker class

#include <OGRE/OgreColourValue.h>

#include <ogre_tools/shape.h>

#include "Displays/Markers/SignObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                const std::string SignObjectOfInterestMarker::ICON_NAME = std::string("debris"); // Loads the file debris.dae in the icon folder
                const Ogre::ColourValue SignObjectOfInterestMarker::ICON_BOX_COLOR = Ogre::ColourValue(0.75, 0.75, 0, 0.50); // Makes the debris yellow and half-transparent

                SignObjectOfInterestMarker::SignObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
                : ObjectOfInterestMarker(sceneMgr, frameTransformer, parent_node)
                {
                    saveOOI(msg);
                }
                
                void SignObjectOfInterestMarker::addToOgre()
                {
                    ObjectOfInterestMarker::addToOgre();
                    
                    Ogre::Vector3 scale;

                    scale.x = 1.25;
                    scale.y = 1.25;
                    scale.z = 1.5; // Half of the polygon is in the ground
                    // Problem with OGRE: if I make it higher, for example 2
                    // meters high, I don't see it anymore from the 2D view

                    ogreBox->setScale(scale);
                }

                const eu_nifti_env::SignObjectOfInterestConstPtr SignObjectOfInterestMarker::getSignObjectOfInterest() const
                {
                    return eu_nifti_env::SignObjectOfInterestConstPtr(&soi);
                }
                
                void SignObjectOfInterestMarker::saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                {
                    ObjectOfInterestMarker::saveOOI(msg);
                    
                    assert(msg->type == eu_nifti_env::ElementOfInterest::TYPE_SIGN);
                    
                    // Does a full copy of the data (or at least I think that it does)
                    soi = eu_nifti_env::SignObjectOfInterest(msg->sign);
                    ooi = &(soi.object);
                }
                
                const std::string& SignObjectOfInterestMarker::getIconName() const
                {
                    return ICON_NAME;
                }
                    
                const Ogre::ColourValue& SignObjectOfInterestMarker::getIconBoxColor() const
                {
                    return ICON_BOX_COLOR;
                }

            }
        }
    }
}

