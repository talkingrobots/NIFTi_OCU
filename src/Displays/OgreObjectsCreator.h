// Benoit 2011-06-16

#include <vector>

#include <eu_nifti_env/Polygon.h>

#ifndef EU_NIFTI_OCU_DISPLAY_OGRE_OBJECTS_CREATOR_H
#define EU_NIFTI_OCU_DISPLAY_OGRE_OBJECTS_CREATOR_H

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                /**
                 * Contains a collection of tools for creating Ogre objects
                 */
                class OgreObjectsCreator
                {
                public:

                    static std::vector<Ogre::ColourValue> colors;

                    static const Ogre::ColourValue& getColor(u_int index);
                    
                    static void createOgreGroundPolygon(const eu_nifti_env::Polygon& polygonROS, Ogre::ManualObject* polygonOgre, const Ogre::ColourValue& color, float defaultHeight, Ogre::Node* node);
                    
                    static void makeCircle(Ogre::ManualObject * circle, double radius, const Ogre::ColourValue& color);

                    static void makeLine(Ogre::ManualObject *line, const Ogre::Vector3& point1, const Ogre::Vector3& point2, const Ogre::ColourValue& color);
                    
                    static void makeSphere(Ogre::ManualObject *sphere, const Ogre::ColourValue& color, const float radius, const int nRings = 16, const int nSegments = 16);

                private:

                    static void initializeColors(); 

                };

            }
        }
    }
}


#endif
