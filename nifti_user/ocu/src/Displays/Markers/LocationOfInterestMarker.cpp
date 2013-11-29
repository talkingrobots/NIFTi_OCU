// Benoit 2010-11-27

//#include <OGRE/OgreSceneManager.h>

#include "Displays/Markers/LocationOfInterestMarker.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                const int LocationOfInterestMarker::HEIGHT_ABOVE_GROUND = 0.10; // Makes the marker appear 10 cm above the ground

                LocationOfInterestMarker::LocationOfInterestMarker(eu_nifti_env::LocationOfInterest& loi, Ogre::SceneManager* sceneMgr)
                : loi(loi)
                , ogreObject(ogre_tools::Shape(ogre_tools::Shape::Cone, sceneMgr))
                , ogreEntity(ogreObject.getEntity())
                {
                    Ogre::Vector3 position(loi.point.x, loi.point.y, loi.point.z);

                    // Makes the marker appear a bit above the ground
                    position.z += HEIGHT_ABOVE_GROUND;

                    ogreObject.setScale(Ogre::Vector3(0.2, 0.3, 0.2)); // Creates a thin cone, 30 cm high
                    ogreObject.setPosition(position);
                    ogreObject.setOrientation(Ogre::Quaternion(sqrt(0.5), sqrt(0.5), 0, 0)); // Turns the cone at 90 deg (since Fuerte)
                    ogreObject.setColor(0.2, 0.6, 0.2, 1); // Green, opaque
                }

                LocationOfInterestMarker::~LocationOfInterestMarker()
                {
                    //sceneMgr->getRootSceneNode()->detachObject(ogreObject); // This does not compile and I'm not even sure that it's necessary
                }

                const std::string& LocationOfInterestMarker::getIdentifier() const
                {
                    return loi.element.name; // This should be unique, although it is probably not checked
                }
                
                IUSARMarker::Type LocationOfInterestMarker::getMarkerType() const
                {
                    return IUSARMarker::ELEMENT_OF_INTEREST;
                }

                const eu_nifti_env::LocationOfInterest* LocationOfInterestMarker::getLocationOfInterest() const
                {
                    return &loi;
                }

                const Ogre::Entity* LocationOfInterestMarker::getOgreIcon() const
                {
                    return ogreEntity;
                }


            }
        }
    }
}

