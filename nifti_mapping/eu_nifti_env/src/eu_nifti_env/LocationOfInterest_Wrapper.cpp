// Benoit 2011-04-07

#include <eu_nifti_env/LocationOfInterest_Wrapper.h>
#include <geometry_msgs/Point.h>

namespace eu_nifti_env
{

    LocationOfInterest_Wrapper::LocationOfInterest_Wrapper(LocationOfInterest* loi)
    : ElementOfInterest_Wrapper(&(loi->element))
    , loi(loi)
    {
//        ElementOfInterest eoi;
//        eoi.uuid = 42;
//        eoi.name = "foo";
//        eoi.type = ElementOfInterest::TYPE_AREA;
//        eoi.sourceType = ElementOfInterest::SOURCE_TYPE_USER;
// 
//        geometry_msgs::Point point;
//        point.x = 1;
//        point.y = 2;
//        point.z = 0;
//
//        LocationOfInterest loi;
//        loi.point = point;
//        loi.element = eoi;

    }

}

