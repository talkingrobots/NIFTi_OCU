// Benoit 2010-11-19

#include "Displays/Markers/shape_marker.h"
#include "Displays/Markers/arrow_marker.h"
#include "Displays/Markers/line_list_marker.h"
#include "Displays/Markers/line_strip_marker.h"
#include "Displays/Markers/sphere_list_marker.h"
#include "Displays/Markers/points_marker.h"
#include "Displays/Markers/text_view_facing_marker.h"
#include "Displays/Markers/mesh_resource_marker.h"
#include "Displays/Markers/triangle_list_marker.h"

#include "Displays/Markers/GenericMarkerCreator.h"

namespace rviz
{

    GenericMarkerCreator* GenericMarkerCreator::theInstance = NULL;

    GenericMarkerCreator::GenericMarkerCreator()
    {
    }

    GenericMarkerCreator* GenericMarkerCreator::instance()
    {
        if (theInstance == NULL)
        {
            theInstance = new GenericMarkerCreator();
        }
        return theInstance;
    }

    rviz::MarkerBase* GenericMarkerCreator::createMarker(const visualization_msgs::Marker::ConstPtr& message, rviz::MarkerDisplay* owner, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* sceneNode) const
    {
        switch (message->type)
        {
            case visualization_msgs::Marker::CUBE:
            case visualization_msgs::Marker::CYLINDER:
            case visualization_msgs::Marker::SPHERE:
                return new rviz::ShapeMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::ARROW:
                return new rviz::ArrowMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::LINE_STRIP:
                return new rviz::LineStripMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::LINE_LIST:
                return new rviz::LineListMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::SPHERE_LIST:
                return new rviz::SphereListMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::CUBE_LIST:
            case visualization_msgs::Marker::POINTS:
                return new rviz::PointsMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::TEXT_VIEW_FACING:
                return new rviz::TextViewFacingMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::MESH_RESOURCE:
                return new rviz::MeshResourceMarker(owner, sceneMgr, frameTransformer, sceneNode);

            case visualization_msgs::Marker::TRIANGLE_LIST:
                return new rviz::TriangleListMarker(owner, sceneMgr, frameTransformer, sceneNode);

            default:
                std::cerr << "Unknown marker type: " << message->type << std::endl;
                return NULL;
        }
    }

}
