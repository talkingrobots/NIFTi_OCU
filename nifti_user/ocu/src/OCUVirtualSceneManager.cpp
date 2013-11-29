// Benoit 2012-07-09

#include "NIFTiConstants.h"
#include "NIFTiROSUtil.h"
#include "NIFTiViewsUtil.h"


#include "Displays/Display.h"
#include "Displays/DistanceDisplay.h"
#include "Displays/GridCellsDisplay.h"
#include "Displays/GridDisplay.h"
#include "Displays/GroundPolygonsDisplay.h"
#include "Displays/InFieldPictureDisplay.h"
#include "Displays/LaserScanDisplay.h"
#include "Displays/MapDisplay.h"
#include "Displays/MarkerDisplay.h"
#include "Displays/NIFTiPoseDisplay.h"
#include "Displays/ObjectOfInterestDisplay.h"
#include "Displays/PathDisplay.h"
#include "Displays/PointCloudDisplay.h"
#include "Displays/PointCloud2Display.h"
#include "Displays/PoseDisplay.h"
#include "Displays/PoseArrayDisplay.h"
#include "Displays/RobotModelDisplay.h"
#include "Displays/TFDisplay.h"
#include "Displays/UAVModelDisplay.h"

#include "Displays/Markers/GenericMarkerCreator.h"

#include "OCUVirtualSceneManager.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            const std::string OCUVirtualSceneManager::ROS_TOPIC_NAV_PLANNED_PATH = "/move_base_node/TrajectoryPlannerROS/global_plan";
            const std::string OCUVirtualSceneManager::ROS_TOPIC_TRAVELED_PATH = "/traveledPath";
            const std::string OCUVirtualSceneManager::ROS_TOPIC_MARKER_TOPO_DECOMPOSITION = "/toposeg/topo_marker";
            const std::string OCUVirtualSceneManager::ROS_TOPIC_FUNCTIONAL_AREAS_CARS = "/func_map/areas";
            const std::string OCUVirtualSceneManager::ROS_TOPIC_POSE_ARRAY_MARKERS = "/poseArray";

            const std::string OCUVirtualSceneManager::ROS_TOPIC_TRAVELED_PATH_MARKERS = "/traveledPath_Markers";
            const std::string OCUVirtualSceneManager::ROS_TOPIC_TIMED_POSE_MARKERS = "/timedPoseMarker";

            OCUVirtualSceneManager::OCUVirtualSceneManager(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::selection::SelectionManager* selectionMgr, ros::CallbackQueue* threadQueue)
            : VirtualSceneManager(sceneMgr, selectionMgr, threadQueue)
            {

            }

            void OCUVirtualSceneManager::createDisplays()
            {
                displays["Grid"] = new rviz::GridDisplay("Grid", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                // I think that this one is not used at all
                //displays[DISPLAY_GRID_CELLS_NAME] = new rviz::GridCellsDisplay(DISPLAY_GRID_CELLS_NAME, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                displays["Map2D"] = new rviz::MapDisplay("Map2D", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                displays["Markers - Detected Objects"] = new eu::nifti::ocu::display::ObjectOfInterestDisplay("Markers - Detected Objects", sceneMgr, frameTransformer, selectionMgr, ros::getGlobalCallbackQueue(), threadQueue);
                
                displays["Markers - In Field Pictures"] = new eu::nifti::ocu::display::InFieldPictureDisplay("Markers - In Field Pictures", sceneMgr, frameTransformer, selectionMgr, ros::getGlobalCallbackQueue(), threadQueue);

                displays["Traveled Path"] = new rviz::PathDisplay("Traveled Path", ROS_TOPIC_TRAVELED_PATH, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                
                if (NIFTiConstants::getRobotType() == "UGV")
                {
                    displays["Distance"] = new eu::nifti::ocu::display::DistanceDisplay("Distance", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                    rviz::LaserScanDisplay *d = new rviz::LaserScanDisplay("Horizontal Laser Scan", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    d->setVisibilityFlags(NIFTiViewsUtil::getVisiblityFlag(d->getName()));
                    displays[d->getName()] = d;

                    displays["Robot Model"] = new rviz::RobotModelDisplay("Robot Model", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                    rviz::PathDisplay* pd = new rviz::PathDisplay("Planned Path", ROS_TOPIC_NAV_PLANNED_PATH, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    pd->setColor(Ogre::ColourValue(1.0f, 0.6f, 0.0f, 1.0f));
                    displays["Planned Path"] = pd;
                }
                else if (NIFTiConstants::getRobotType() == "UAV")
                {
                    displays["UAV Model"] = new eu::nifti::ocu::display::UAVModelDisplay("UAV Model", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                }

                bool show3DMap;
                if (NIFTiROSUtil::getParam("show3DMap", show3DMap) && show3DMap)
                {
                    if (NIFTiConstants::getRobotType() == "UGV")
                    {
                        rviz::PointCloud2Display *d;

                        // This adds the full environment point cloud
                        d = new rviz::PointCloud2Display("Point Cloud 2 - Map", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                        displays[d->getName()] = d;

                        d->setTopic("/point_map");
                        d->setStyle(ogre_tools::PointCloud::RM_POINTS);
                        d->setAlpha(1);
                        d->setDecayTime(0);
                        d->setXYZTransformer("XYZ");
                        d->setColorTransformer("RGB8");

                        d->setVisibilityFlags(NIFTiViewsUtil::getVisiblityFlag(d->getName()));


                        // This adds the point cloud for navigation
                        d = new rviz::PointCloud2Display("Point Cloud 2 - Dynamic", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                        displays[d->getName()] = d;

                        d->setTopic("/dynamic_point_cloud");
                        d->setStyle(ogre_tools::PointCloud::RM_POINTS);
                        d->setAlpha(1);
                        d->setDecayTime(0);
                        d->setXYZTransformer("XYZ");
                        d->setColorTransformer("Intensity");

                        d->setVisibilityFlags(NIFTiViewsUtil::getVisiblityFlag(d->getName()));
                    }
                    else if (NIFTiConstants::getRobotType() == "UAV")
                    {
                        rviz::PointCloudDisplay *d = new rviz::PointCloudDisplay("Point Cloud UAV", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                        d->setVisibilityFlags(NIFTiViewsUtil::getVisiblityFlag(d->getName()));
                        displays[d->getName()] = d;
                    }
                }

                bool showDebugDisplays;
                if (NIFTiROSUtil::getParam("showDebugDisplays", showDebugDisplays) && showDebugDisplays)
                {
                    displays["Ground Polygons"] = new rviz::GroundPolygonsDisplay("Ground Polygons", ROS_TOPIC_FUNCTIONAL_AREAS_CARS, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    displays["TF"] = new rviz::TFDisplay("TF", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    displays["Markers - Topological Decomposition"] = new rviz::MarkerDisplay("Markers - Topological Decomposition", ROS_TOPIC_MARKER_TOPO_DECOMPOSITION, sceneMgr, frameTransformer, rviz::GenericMarkerCreator::instance(), ros::getGlobalCallbackQueue(), threadQueue);

                    // Used to generate maps for the Y2 OCU report
                    displays["Markers - Timed Pose"] = new rviz::MarkerDisplay("Markers - Timed Pose", ROS_TOPIC_TIMED_POSE_MARKERS, sceneMgr, frameTransformer, rviz::GenericMarkerCreator::instance(), ros::getGlobalCallbackQueue(), threadQueue);

                    displays["Pose Array"] = new rviz::PoseArrayDisplay("Pose Array", ROS_TOPIC_POSE_ARRAY_MARKERS, sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                }

                bool showGapDisplays;
                if (NIFTiROSUtil::getParam("showGapDisplays", showGapDisplays) && showGapDisplays)
                {
                    // Displays gaps and arrows for the start and end pose of traversal (ICMI 2012)
                    displays["ICMI_2012_START_POSE"] = new rviz::PoseDisplay("ICMI_2012_START_POSE", "/holec/gui_/GUI_init_pose", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    displays["ICMI_2012_END_POSE"] = new rviz::PoseDisplay("ICMI_2012_END_POSE", "/holec/gui_/GUI_final_pose", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);

                    rviz::GroundPolygonsDisplay* gpd = new rviz::GroundPolygonsDisplay("ICMI_2012_GAPS", "/holec/gui_/GUI_GAP", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    gpd->setFillPolygons(false);
                    displays["ICMI_2012_GAPS"] = gpd;

                    rviz::PointCloud2Display* d = new rviz::PointCloud2Display("ICMI_2012_GAPS_PC", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    d->setTopic("/holec/hole_contour");
                    d->setDecayTime(0);
                    displays["ICMI_2012_GAPS_PC"] = d;

                    d = new rviz::PointCloud2Display("ICMI_2012_PC_ROMA", sceneMgr, frameTransformer, ros::getGlobalCallbackQueue(), threadQueue);
                    d->setTopic("/static_point_cloud_ROMA");
                    d->setDecayTime(0);
                    displays["ICMI_2012_PC_ROMA"] = d;
                }

            }

            void OCUVirtualSceneManager::onSelectionManagerDeleted()
            {
                selectionMgr = NULL;

                // Tells the OOI Display to avoid telling the selection manager when the markers eventually get deleted
                ((eu::nifti::ocu::display::ObjectOfInterestDisplay*) displays["Markers - Detected Objects"])->onSelectionManagerDeleted();
                ((eu::nifti::ocu::display::InFieldPictureDisplay*) displays["Markers - In Field Pictures"])->onSelectionManagerDeleted();
            }

        }
    }
}
