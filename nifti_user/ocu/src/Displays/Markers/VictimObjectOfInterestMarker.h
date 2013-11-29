// Benoit 2011-06-10 

#ifndef EU_NIFTI_OCU_DISPLAY_VICTIM_OOI_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_VICTIM_OOI_MARKER_H

#include <boost/shared_ptr.hpp>

#include <eu_nifti_env/ObjectOfInterest.h>
#include <eu_nifti_env/VictimObjectOfInterest.h>

#include "Displays/Markers/ObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                class VictimObjectOfInterestMarker : public ObjectOfInterestMarker
                {
                public:
                    VictimObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node);

                    const eu_nifti_env::VictimObjectOfInterestConstPtr getVictimObjectOfInterest() const;
                    
                    void addToOgre();

                protected:
                    
                    virtual void saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);

                    const std::string& getIconName() const;
                    const Ogre::ColourValue& getIconBoxColor() const;
                    
                    eu_nifti_env::VictimObjectOfInterest voi;
                    
                    static const std::string ICON_NAME;
                    static const Ogre::ColourValue ICON_BOX_COLOR;

                };

                typedef boost::shared_ptr<VictimObjectOfInterestMarker> VictimObjectOfInterestMarkerPtr;

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_VICTIM_OOI_MARKER_H


