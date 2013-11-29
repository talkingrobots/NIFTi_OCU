// Benoit 2010-11-27

#ifndef EU_NIFTI_OCU_DISPLAY_LOI_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_LOI_MARKER_H

#include <ogre_tools/shape.h>

#include <eu_nifti_env/LocationOfInterest.h>

#include "Displays/Markers/IUSARMarker.h"

namespace Ogre
{
    class SceneManager;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                /**
                 * Contains two items: 1) the actual LocationOfInterest (the 
                 * info) and 2) the OGRE object (the representation)
                 */
                class LocationOfInterestMarker : public IUSARMarker
                {
                    
                public:
                    LocationOfInterestMarker(eu_nifti_env::LocationOfInterest& loi, Ogre::SceneManager* sceneMgr);
                    ~LocationOfInterestMarker();
                    
                    const std::string& getIdentifier() const;
                    
                    IUSARMarker::Type getMarkerType() const;

                    const eu_nifti_env::LocationOfInterest* getLocationOfInterest() const;

                    const Ogre::Entity* getOgreIcon() const;

                protected:
                    
                    eu_nifti_env::LocationOfInterest loi;
                    ogre_tools::Shape ogreObject;
                    Ogre::Entity* ogreEntity; // This is the entity owned/represented by ogreObject. It's just an extra link for practical reasons.
                    
                    static const int HEIGHT_ABOVE_GROUND;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_LOI_MARKER_H


