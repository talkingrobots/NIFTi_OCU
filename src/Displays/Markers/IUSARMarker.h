// Benoit 2013-07-22 

#ifndef EU_NIFTI_OCU_DISPLAY_I_USAR_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_I_USAR_MARKER_H

namespace Ogre
{
    class Entity;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                class IUSARMarker
                {
                public:
                    
                    enum Type { ELEMENT_OF_INTEREST, ANNOTATED_PICTURE };

                    virtual const std::string& getIdentifier() const = 0;
                    
                    virtual Type getMarkerType() const = 0;
                    
                    virtual void onSelect() {};
                    virtual void onDeselect() {};
                    
                    virtual const Ogre::Entity* getOgreIcon() const = 0;
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_I_USAR_MARKER_H


