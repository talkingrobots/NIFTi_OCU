// Benoit 2011-06-10 

#ifndef EU_NIFTI_OCU_DISPLAY_SIGN_OOI_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_SIGN_OOI_MARKER_H

#include <boost/shared_ptr.hpp>

#include <eu_nifti_env/SignObjectOfInterest.h>

#include "Displays/Markers/ObjectOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                class SignObjectOfInterestMarker : public ObjectOfInterestMarker
                {
                public:
                    SignObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node);

                    const eu_nifti_env::SignObjectOfInterestConstPtr getSignObjectOfInterest() const;
                    
                    void addToOgre();

                protected:
                    
                    virtual void saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);

                    const std::string& getIconName() const;
                    const Ogre::ColourValue& getIconBoxColor() const;
                    
                    eu_nifti_env::SignObjectOfInterest soi;
                    
                    static const std::string ICON_NAME;
                    static const Ogre::ColourValue ICON_BOX_COLOR;
                    
                };

                typedef boost::shared_ptr<SignObjectOfInterestMarker> SignObjectOfInterestMarkerPtr;

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_SIGN_OOI_MARKER_H


