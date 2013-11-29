include_directories(.)

# We a library the C++ code (to be used by an executable below
rosbuild_add_library(${PROJECT_NAME} 
                                    AnnotatedPicUtil.cpp
				    InFieldPicsManager.cpp
                                    NIFTiConstants.cpp
                                    NIFTiROSOgreUtil.cpp
                                    NIFTiROSUtil.cpp
                                    NIFTiViewsUtil.cpp
                                    OCUMessagePublisherThread.cpp
                                    OCUVirtualSceneManager.cpp
                                    PictureDisplayUtil.cpp
				    PictureMetadataUtil.cpp
				    RobotsStatusesManager.cpp
                                    UUIDsManager.cpp
                                    VirtualSceneManager.cpp
                                    VisualizationManager.cpp
                                    VisualizationManagerForBlankScreen.cpp
                                    VisualizationManagerForCamera.cpp
                                    VisualizationManagerFor3DScene.cpp
                                    VisualizationManagerForPhoto.cpp
                                    WXLogROSOut.cpp

                                    Displays/Markers/arrow_marker.cpp
                                    Displays/Markers/line_list_marker.cpp
                                    Displays/Markers/line_strip_marker.cpp
                                    Displays/Markers/marker_base.cpp
                                    Displays/Markers/mesh_loader.cpp
                                    Displays/Markers/mesh_resource_marker.cpp
                                    Displays/Markers/InFieldPictureMarker.cpp
                                    Displays/Markers/LocationOfInterestMarker.cpp
                                    Displays/Markers/ObjectOfInterestMarker.cpp
                                    Displays/Markers/CarObjectOfInterestMarker.cpp
                                    Displays/Markers/SignObjectOfInterestMarker.cpp
                                    Displays/Markers/VictimObjectOfInterestMarker.cpp
                                    Displays/Markers/points_marker.cpp
                                    Displays/Markers/shape_marker.cpp
                                    Displays/Markers/sphere_list_marker.cpp
                                    Displays/Markers/text_view_facing_marker.cpp
                                    Displays/Markers/triangle_list_marker.cpp
                                    Displays/Markers/GenericMarkerCreator.cpp
                                    Displays/Markers/OOIMarkerFactory.cpp

                                    Displays/Transformers/PointCloud/AxisColorPCTransformer.cpp
                                    Displays/Transformers/PointCloud/FlatColorPCTransformer.cpp
                                    Displays/Transformers/PointCloud/IntensityPCTransformer.cpp
                                    Displays/Transformers/PointCloud/RGB8PCTransformer.cpp
                                    Displays/Transformers/PointCloud/RGBF32PCTransformer.cpp
                                    Displays/Transformers/PointCloud/XYZPCTransformer.cpp
                                    Displays/Transformers/FrameTransformer.cpp

                                    Displays/Display.cpp
                                    Displays/CameraDisplay.cpp
                                    Displays/DistanceDisplay.cpp
                                    Displays/GridCellsDisplay.cpp
                                    Displays/GridDisplay.cpp
                                    Displays/GroundPolygonsDisplay.cpp
                                    Displays/ImageDisplay.cpp
                                    Displays/InFieldPictureDisplay.cpp
                                    Displays/LaserScanDisplay.cpp
                                    Displays/MapDisplay.cpp
                                    Displays/MarkerDisplay.cpp
                                    Displays/NIFTiPoseDisplay.cpp
                                    Displays/ObjectOfInterestDisplay.cpp
                                    Displays/OgreObjectsCreator.cpp
                                    Displays/PathDisplay.cpp
                                    Displays/PhotoDisplay.cpp
                                    Displays/PointCloudBase.cpp
                                    Displays/PointCloudDisplay.cpp
                                    Displays/PointCloud2Display.cpp
                                    Displays/PoseArrayDisplay.cpp
                                    Displays/PoseDisplay.cpp
                                    Displays/RobotModelDisplay.cpp
                                    Displays/ROSImageListener.cpp
                                    Displays/ROSImageTexture.cpp
                                    Displays/TFDisplay.cpp
                                    Displays/UAVModelDisplay.cpp

                                    Panels/DialoguePanel.cpp
				    Panels/ElementOfInterestPanel.cpp
				    Panels/FilmstripPanel.cpp
                                    Panels/FilmstripPanelForInFieldPics.cpp
                                    Panels/FilmstripPanelForViewTypes.cpp
                                    Panels/MotorControlPanel.cpp
                                    Panels/OCUFrame.cpp
                                    Panels/PictureInfoPanel.cpp
                                    Panels/PlanningPanel.cpp
                                    Panels/PullHandle.cpp
                                    Panels/QuadVizPanel.cpp
                                    Panels/RenderPanel.cpp
                                    Panels/RightPanel.cpp
                                    Panels/SimplifiedDialoguePanel.cpp
                                    Panels/SplashScreen.cpp
                                    Panels/ThreeHandlePanel.cpp
                                    Panels/TopPanel.cpp
                                    Panels/UAVControlPanel.cpp
                                    Panels/UGVActuatorsControlPanel.cpp
                                    Panels/VisualizationFrame.cpp
                                    Panels/WaitForMasterDialog.cpp
                                    Panels/WaitForMasterThread.cpp

                                    Robot/Robot.cpp
                                    Robot/RobotLink.cpp
                                    Robot/TFLinkUpdater.cpp

                                    Selection/SelectionManager.cpp
                                    Selection/LocationSelectionManager.cpp
                                    Selection/MarkerSelectionManager.cpp

                                    Tools/Tool.cpp
                                    Tools/StandardOCUTool.cpp
                                    Tools/ClickTool.cpp
                                    Tools/DebugTool.cpp
                                    Tools/GoalTool.cpp
                                    Tools/GoToTool.cpp
                                    Tools/MoveTool.cpp
                                    Tools/PhotoTool.cpp
                                    Tools/PoseTool.cpp

                                    ViewControllers/ViewController.cpp
                                    ViewControllers/ArmCamViewController.cpp
                                    ViewControllers/BlankViewController.cpp
                                    ViewControllers/CameraViewController.cpp
                                    ViewControllers/OrthogonalViewController.cpp
                                    ViewControllers/PerspectiveViewController.cpp
                                    ViewControllers/PhotoViewController.cpp
                                    ViewControllers/Chase2DViewController.cpp
                                    ViewControllers/Chase3DViewController.cpp
                                    ViewControllers/FirstPersonViewController.cpp
                                    ViewControllers/FixedCamViewController.cpp
                                    ViewControllers/Map2DViewController.cpp
                                    ViewControllers/Map3DViewController.cpp
                                    ViewControllers/PTZCamViewController.cpp
                                    ViewControllers/VirtualPTZViewController.cpp
                                    ViewControllers/ViewControllerFactory.cpp

                                    )

target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} eu_nifti_env_msg_ros) 
# eu_nifti_env_msg_ros is necessary because I use a Util file in that package

rosbuild_link_boost(${PROJECT_NAME} thread signals filesystem system program_options)

# Creates the executable that will use the library
rosbuild_add_executable(theOCU AppOCU.cpp)
target_link_libraries(theOCU ${PROJECT_NAME} assimp ${wxWidgets_LIBRARIES})
set_target_properties(theOCU
                      PROPERTIES OUTPUT_NAME "OCU"
                      PREFIX "")

#rosbuild_add_executable(appTestConsecutivePublishers AppTestConsecutivePublishers.cpp)

#rosbuild_add_executable(appTestNameWithSlash AppTestNameWithSlash.cpp)

#rosbuild_add_executable(eoi_listener eoi_listener.cpp)

#rosbuild_add_executable(appTestCallbackSubscription AppTestCallbackSubscription.cpp)

#rosbuild_add_executable(basic_shapes ../tests/basic_shapes.cpp)

#rosbuild_add_executable(basic_shapes2 ../tests/basic_shapes2.cpp)

#rosbuild_add_executable(FakeNIFTiObjectGenerator ../tests/FakeNIFTiObjectGenerator.cpp)

#rosbuild_add_executable(FakeOOIGenerator ../tests/FakeOOIGenerator.cpp)

#rosbuild_add_executable(FakeTFBroadcaster2 ../tests/FakeTFBroadcaster2.cpp)

#rosbuild_add_executable(FakeTFBroadcaster3 ../tests/FakeTFBroadcaster3.cpp)

#rosbuild_add_executable(FakeFlippersCurrentsBroadcaster ../tests/FakeFlippersCurrentsBroadcaster.cpp)

#rosbuild_add_executable(FakeDialogueGenerator ../tests/FakeDialogueGenerator.cpp)
