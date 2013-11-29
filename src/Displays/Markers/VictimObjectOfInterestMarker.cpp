// Benoit 2010-11-17 Based on the RVIZ MeshResourceMarker class

#include <OGRE/OgreColourValue.h>

#include <ogre_tools/shape.h>

#include "Displays/Markers/VictimObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                const std::string VictimObjectOfInterestMarker::ICON_NAME = std::string("victim"); // Loads the file victim.dae in the icon folder
                const Ogre::ColourValue VictimObjectOfInterestMarker::ICON_BOX_COLOR = Ogre::ColourValue(0.75, 0, 0, 0.50); // Makes the victim red and half-transparent

                VictimObjectOfInterestMarker::VictimObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node)
                : ObjectOfInterestMarker(sceneMgr, frameTransformer, parent_node)
                {
                    saveOOI(msg);
                }
                
                void VictimObjectOfInterestMarker::addToOgre()
                {
                    ObjectOfInterestMarker::addToOgre();
                    
                    Ogre::Vector3 scale;

                    scale.x = 1.5;
                    scale.y = 1;
                    scale.z = 1; // Half of the polygon is in the ground
                    // Problem with OGRE: if I make it higher, for example 2
                    // meters high, I don't see it anymore from the 2D view

                    ogreBox->setScale(scale);
                }

                const eu_nifti_env::VictimObjectOfInterestConstPtr VictimObjectOfInterestMarker::getVictimObjectOfInterest() const
                {
                    return eu_nifti_env::VictimObjectOfInterestConstPtr(&voi);
                }
                
                void VictimObjectOfInterestMarker::saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                {
                    ObjectOfInterestMarker::saveOOI(msg);
                    
                    assert(msg->type == eu_nifti_env::ElementOfInterest::TYPE_VICTIM);
                    
                    // Does a full copy of the data (or at least I think that it does)
                    voi = eu_nifti_env::VictimObjectOfInterest(msg->victim);
                    ooi = &(voi.object);
                }
                
                const std::string& VictimObjectOfInterestMarker::getIconName() const
                {
                    return ICON_NAME;
                }
                    
                const Ogre::ColourValue& VictimObjectOfInterestMarker::getIconBoxColor() const
                {
                    return ICON_BOX_COLOR;
                }

            }
        }
    }
}

