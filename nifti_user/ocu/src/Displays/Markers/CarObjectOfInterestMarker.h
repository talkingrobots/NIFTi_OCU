// Benoit 2011-06-10 

#ifndef EU_NIFTI_OCU_DISPLAY_CAR_OOI_MARKER_H
#define EU_NIFTI_OCU_DISPLAY_CAR_OOI_MARKER_H

#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>

#include <eu_nifti_env/CarObjectOfInterest.h>

#include "Displays/Markers/ObjectOfInterestMarker.h"

namespace Ogre
{
    class ManualObject;
    class ColourValue;
}

//namespace eu_nifti_env
//{
//    class Polygon;
//}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                class CarObjectOfInterestMarker : public ObjectOfInterestMarker
                {
                public:
                    CarObjectOfInterestMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* parent_node);

                    const eu_nifti_env::CarObjectOfInterestConstPtr getCarObjectOfInterest() const;

                    void addToOgre();
                    void modifyInOgre(const std::string& frame_id, ros::Time stamp, const geometry_msgs::Pose& pose);

                protected:

                    void saveOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg);

                    void modifyPolygons();
                    void clearPolygons();
                    
                    const std::string& getIconName() const;
                    const Ogre::ColourValue& getIconBoxColor() const;

                    eu_nifti_env::CarObjectOfInterest coi;

                    std::set<Ogre::ManualObject*>* polygons;
                    // For some reason, if I use a set instead of a set*, I get this error:
                    // "glibc detected" "corrupted double-linked list"

                    static void createOgreGroundPolygon(const eu_nifti_env::Polygon& polygonROS, Ogre::ManualObject* polygonOgre, const Ogre::ColourValue* color, float defaultHeight = 0, Ogre::Node* node = NULL);

                    static const std::string ICON_NAME;
                    static const Ogre::ColourValue ICON_BOX_COLOR;
                    
                    static const float DEFAULT_HEIGHT_FUNC_AREAS;

                };

                typedef boost::shared_ptr<CarObjectOfInterestMarker> CarObjectOfInterestMarkerPtr;

            }
        }
    }
}

#endif // EU_NIFTI_OCU_DISPLAY_CAR_OOI_MARKER_H


