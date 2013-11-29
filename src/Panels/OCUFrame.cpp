// Benoit 2012-06-29

#include <string>
#include <vector>

#include <wx/aui/aui.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include <eu_nifti_env_msg_ros/RequestForUUIDs.h>
#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>

#include <eu_nifti_ocu_msg_ros/SelectionMessage.h>

#include <nifti_ocu_msgs/ToolActivation.h>
#include <nifti_ocu_msgs/ViewControl.h>

#include "Displays/Markers/InFieldPictureMarker.h"
#include "Displays/Markers/IUSARMarker.h"
#include "Displays/Markers/ObjectOfInterestMarker.h"

#include "Displays/Transformers/FrameTransformer.h"

#include "Tools/ClickTool.h"
#include "Tools/DebugTool.h"
#include "Tools/GoalTool.h"
#include "Tools/GoToTool.h"
#include "Tools/MoveTool.h"
#include "Tools/PhotoTool.h"

#include "Panels/FilmstripPanelForViewTypes.h"
#include "Panels/FilmstripPanelForInFieldPics.h"
#include "Panels/RightPanel.h"
#include "Panels/QuadVizPanel.h"
#include "Panels/RenderPanel.h"
#include "Panels/SplashScreen.h"
#include "Panels/TopPanel.h"
#include "Panels/ThreeHandlePanel.h"
//#include "Panels/UAVControlPanel.h" // Temporarily disabled until the UAV is controllable by distance

#include "Selection/SelectionManager.h"

#include "InFieldPicsManager.h"
#include "NIFTiConstants.h"
#include "NIFTiROSUtil.h"
#include "OCUMessagePublisherThread.h"
#include "UUIDsManager.h"

#include "OCUVirtualSceneManager.h"
#include "VisualizationManagerForBlankScreen.h"
#include "VisualizationManagerForCamera.h"
#include "VisualizationManagerFor3DScene.h"
#include "VisualizationManagerForPhoto.h"

#include "Panels/OCUFrame.h"
#include "FilmstripPanelForInFieldPics.h"

using namespace std;
using namespace eu::nifti::ocu;
using namespace eu::nifti::ocu::gui;

namespace rviz
{
    const char* OCUFrame::TOPIC_TOOL_ACTIVATION = "ocu/toolActivation";
    const char* OCUFrame::TOPIC_VIEW_CONTROL = "ocu/viewControl";

    const int OCUFrame::WINDOW_WIDTH = 1280;
    const int OCUFrame::WINDOW_HEIGHT = 800;

    OCUFrame::OCUFrame()
    : VisualizationFrame(wxT("NIFTi Operator Control Unit"), wxDefaultPosition, wxSize(WINDOW_WIDTH, WINDOW_HEIGHT))
    , rightPanel(NULL)
    , threeHandlePanel(NULL)
    , viewTypeWasRecentlySelected(false)
    , vizModeWasRecentlySelected(false)
    , uuidMgr(NULL)
    , draggingInProgress(false)
    {
    }

    OCUFrame::~OCUFrame()
    {
        //ROS_INFO("IN OCUFrame::~OCUFrame()");

        if (selectionMgr != NULL)
        {
            delete selectionMgr;
            ((eu::nifti::ocu::OCUVirtualSceneManager*)virtualSceneManager)->onSelectionManagerDeleted();
        }

        toolBarForTools->Disconnect(wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler(OCUFrame::onToolClicked), NULL, this);
        Disconnect(wxEVT_SIZE, wxSizeEventHandler(OCUFrame::OnSize));

        if (ocuMsgPubThread != NULL)
        {
            ocuMsgPubThread->stopPublishing();
            ocuMsgPubThread->Wait();
            ocuMsgPubThread->Delete();
            delete ocuMsgPubThread;
        }

        if (uuidMgr != NULL)
        {
            uuidMgr->stopManaging();
            uuidMgr->Wait();
            uuidMgr->Delete();
            delete uuidMgr;
        }

        eu::nifti::ocu::InFieldPicsManager::instance()->unInit();

        //ROS_INFO("OUT OCUFrame::~OCUFrame()");
    }

    void OCUFrame::OnSize(wxSizeEvent& event)
    {
        if (rightPanel != NULL)
        {
            forceRightPanelToExpand();
        }

        if (threeHandlePanel != NULL)
            threeHandlePanel->Refresh();

        event.Skip();
    }

    void OCUFrame::preInit()
    {
        publisherToolActivation = NIFTiROSUtil::getNodeHandle()->advertise<nifti_ocu_msgs::ToolActivation > (TOPIC_TOOL_ACTIVATION, 1);
        publisherViewControl = NIFTiROSUtil::getNodeHandle()->advertise<nifti_ocu_msgs::ViewControl > (TOPIC_VIEW_CONTROL, 1);
        ocuMsgPubThread = new OCUMessagePublisherThread();

        Connect(wxEVT_SIZE, wxSizeEventHandler(OCUFrame::OnSize));
    }

    void OCUFrame::createRenderPanels()
    {
        renderPanels.at(0) = new RenderPanel(multiVizPanel, 0, sceneMgr, this);
        renderPanels.at(1) = new RenderPanel(multiVizPanel, 1, sceneMgr, this);
        renderPanels.at(2) = new RenderPanel(multiVizPanel, 2, sceneMgr, this);
        renderPanels.at(3) = new RenderPanel(multiVizPanel, 3, sceneMgr, this);
    }

    void OCUFrame::createSceneManagers()
    {
        selectionMgr = new eu::nifti::ocu::selection::SelectionManager(this);
        selectionMgr->initialize();

        virtualSceneManager = new eu::nifti::ocu::OCUVirtualSceneManager(sceneMgr, selectionMgr, &threaded_queue_);
    }

    void OCUFrame::createVizualizationManagers()
    {
        vizMgrs.at(0) = new VisualizationManagerFor3DScene(sceneMgr, virtualSceneManager->getFrameTransformer(), selectionMgr, renderPanels.at(0), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(0)));

        vizMgrs.at(1) = new VisualizationManagerForCamera(sceneMgr, new rviz::FrameTransformer(), selectionMgr, renderPanels.at(1), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(1)));

        if (NIFTiConstants::getRobotType() == "P3AT")
        {
            vizMgrs.at(2) = new VisualizationManagerFor3DScene(sceneMgr, virtualSceneManager->getFrameTransformer(), selectionMgr, renderPanels.at(2), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(2)));
        }
        else
        {
            vizMgrs.at(2) = new VisualizationManagerForCamera(sceneMgr, new rviz::FrameTransformer(), selectionMgr, renderPanels.at(2), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(2)));
        }

        vizMgrs.at(3) = new VisualizationManagerFor3DScene(sceneMgr, virtualSceneManager->getFrameTransformer(), selectionMgr, renderPanels.at(3), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(3)));

        //vizMgrs.at(3) = new VisualizationManagerForBlankScreen(renderPanels.at(3), &(ocuMsgPubThread->guiStatus.viewParams.at(3)));
    }

    void OCUFrame::createPanels()
    {
        // MIDDLE
        multiVizPanel->setMode(VIZ_MODE_QUAD, renderPanels.at(0), renderPanels.at(1), renderPanels.at(2), renderPanels.at(3));
        ocuMsgPubThread->guiStatus.vizMode = VIZ_MODE_QUAD;
        auiManager->AddPane(multiVizPanel, wxAuiPaneInfo().CenterPane().Layer(0).Name(wxT("MultiVizPanel")));


        // BOTTOM
        threeHandlePanel = new ThreeHandlePanel(this, auiManager);
        auiManager->AddPane(threeHandlePanel, wxAuiPaneInfo().ToolbarPane().Bottom().Layer(1).MinSize(-1, threeHandlePanel->GetMinHeight()).Floatable(false).Gripper(false).Name(wxT("ThreeHandlePanel")));

        FilmstripPanelForViewTypes* pnlMaps = new FilmstripPanelForViewTypes(threeHandlePanel, this);
        FilmstripPanelForViewTypes* pnlCameras = new FilmstripPanelForViewTypes(threeHandlePanel, this);
        pnlInFieldPics = new FilmstripPanelForInFieldPics(threeHandlePanel, selectionMgr);
        pnlFilmstrips[0] = pnlMaps;
        pnlFilmstrips[1] = pnlCameras;
        pnlFilmstrips[2] = pnlInFieldPics;

        threeHandlePanel->setPanels(pnlMaps, pnlCameras, pnlInFieldPics);

        customizeFilmstripPanelForViewTypes(pnlMaps, pnlCameras);


        // LEFT
        toolBarForTools = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER | wxTB_VERTICAL);
        auiManager->AddPane(toolBarForTools, wxAuiPaneInfo().ToolbarPane().Left().Layer(2).Floatable(false).Gripper(false).Name(wxT("Tools")));
        toolBarForTools->Connect(wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler(OCUFrame::onToolClicked), NULL, this);

        // RIGHT
        bool showRightPanel = false;
        if (NIFTiROSUtil::getParam("showRightPanel", showRightPanel) && showRightPanel)
        {
            rightPanel = new RightPanel(this, this);
            auiManager->AddPane(rightPanel, wxAuiPaneInfo().Right().Layer(2).Floatable(false).CloseButton(false).Resizable(false).Name(wxT("Right Panel")));
        }


        // TOP
        topPanel = new TopPanel(this, this);
        auiManager->AddPane(topPanel, wxAuiPaneInfo().ToolbarPane().Top().Layer(3).MinSize(-1, NIFTiConstants::MIN_BUTTON_SIZE + 15).Floatable(false).Gripper(false).Name(wxT("TOP PANEL")));
        // I have to add approx 15 pixels because it otherwise gets squished on the MacBook Pros


        // Temporarily disabled until the UAV is stable enough
        //uavControlPanel = new eu::nifti::ocu::gui::UAVControlPanel(this); 
        //auiManager->AddPane(uavControlPanel, wxAuiPaneInfo().Right().Floatable(false).CloseButton(false).Resizable(false).BestSize(256, 256).MinSize(256, 256).MaxSize(256, 256).Name(wxT("UAV Control")).Caption(wxT("UAV Control")));

        //Connect(wxEVT_SIZE, wxSizeEventHandler(OCUFrame::OnSize));
    }

    void OCUFrame::createTools()
    {
        // Creates the tools that allow the user to interact with the 3D view
        addTool(new eu::nifti::ocu::tools::ClickTool(nifti_ocu_msgs::ToolActivation::TOOL_CLICK, "Click", "CursorStandardArrow", &publisherViewControl, selectionMgr), true);
        addTool(new eu::nifti::ocu::tools::GoToTool(nifti_ocu_msgs::ToolActivation::TOOL_GO_TO, "Go To", "CursorGoTo", &publisherViewControl, virtualSceneManager));
        addTool(new eu::nifti::ocu::tools::PhotoTool(nifti_ocu_msgs::ToolActivation::TOOL_PHOTO, "Photo", "CursorPhoto", &publisherViewControl, this));
        defaultTool = activeTool;

        // Adds tools used only for debugging
        bool showDebugTools = false;
        if (NIFTiROSUtil::getParam("showDebugTools", showDebugTools) && showDebugTools)
        {
            addTool(new GoalTool(nifti_ocu_msgs::ToolActivation::TOOL_NAV_GOAL_RVIZ, "Nav Goal RVIZ", "CursorRVIZArrow38", sceneMgr));
            addTool(new eu::nifti::ocu::tools::DebugTool(nifti_ocu_msgs::ToolActivation::TOOL_DEBUG, "Debug Info", "CursorQuestion", &publisherViewControl));


            // For the 2 bitmaps for non-developed tools
            std::string fullPath;
            wxImage img;
            bool success;

            // Adds buttons for tools that are not yet developed
            {
                fullPath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/CursorObserve38.png";
                success = img.LoadFile(wxString::FromAscii(fullPath.c_str()), wxBITMAP_TYPE_PNG);
                assert(success);

                toolBarForTools->AddRadioTool(666, wxEmptyString, wxBitmap(img));

                wxAuiPaneInfo& pane = auiManager->GetPane(toolBarForTools);
                pane.MinSize(toolBarForTools->GetSize());
                auiManager->Update();
            }
            {
                fullPath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/CursorExplore38.png";
                success = img.LoadFile(wxString::FromAscii(fullPath.c_str()), wxBITMAP_TYPE_PNG);
                assert(success);

                toolBarForTools->AddRadioTool(999, wxEmptyString, wxBitmap(img));
                wxAuiPaneInfo& pane = auiManager->GetPane(toolBarForTools);
                pane.MinSize(toolBarForTools->GetSize());
                auiManager->Update();
            }

        }
    }

    void OCUFrame::setDefaultViewTypes()
    {
        assert(vizMgrs.at(0) && vizMgrs.at(1) && vizMgrs.at(2) && vizMgrs.at(3));

        // Sets the default view types in accordance to the robot type

        std::vector<ViewType> defaultViews;

        if (NIFTiConstants::getRobotType() == "UGV")
        {
            defaultViews.push_back(VIEW_TYPE_CHASE_2D);
            defaultViews.push_back(VIEW_TYPE_VIRTUAL_PTZ_CAM);
            defaultViews.push_back(VIEW_TYPE_OMNI_CAM);
            defaultViews.push_back(VIEW_TYPE_CHASE_3D);
        }
        else if (NIFTiConstants::getRobotType() == "UAV")
        {
            defaultViews.push_back(VIEW_TYPE_MAP_2D);
            defaultViews.push_back(VIEW_TYPE_UAV_CAM_FRONT);
            defaultViews.push_back(VIEW_TYPE_UAV_CAM_DOWN);
            defaultViews.push_back(VIEW_TYPE_CHASE_3D);
        }
        else if (NIFTiConstants::getRobotType() == "P3AT")
        {
            defaultViews.push_back(VIEW_TYPE_MAP_2D);
            defaultViews.push_back(VIEW_TYPE_KINECT_CAM);
            defaultViews.push_back(VIEW_TYPE_CHASE_2D);
            defaultViews.push_back(VIEW_TYPE_CHASE_3D);
        }

        for (u_int viewIndex = 0; viewIndex < defaultViews.size(); viewIndex++)
        {
            try
            {
                vizMgrs.at(viewIndex)->setViewType(defaultViews.at(viewIndex));
            }
            catch (image_transport::TransportLoadException& ex)
            {
                std::cerr << "Visualization Panel #" << viewIndex << " will not display the video feed" << std::endl;
                // Continue with the rest, and leave the viz panel with the image "No Image"
            }
            catch (eu::nifti::misc::ExceptionWithString& ex)
            {
                std::cerr << "Problem with the initialization of Visualization Panel #" << viewIndex << ": " << ex.what() << std::endl;
                // Continue with the rest, and leave the viz panel in an unknown state
            }
        }

    }

    void OCUFrame::postInit()
    {
        if (rightPanel != NULL)
        {
            forceRightPanelToExpand(); // This also is a hack because wxWidgets is archaic
        }

        // Ensures that the info published about the OCU is initialized correctly
        // These lines are redundant, but it seems like it does not work without them
        updateOCUinfoViewType(0);
        updateOCUinfoViewType(1);
        updateOCUinfoViewType(2);
        updateOCUinfoViewType(3);

        ocuMsgPubThread->Create();
        ocuMsgPubThread->Run();

        // Starts the class that will ensure the availability of UUIDs
        uuidMgr = UUIDsManager::getInstance();
        uuidMgr->Create();
        uuidMgr->Run();

        eu::nifti::ocu::InFieldPicsManager::instance()->init();
    }

    void OCUFrame::addTool(eu::nifti::ocu::tools::Tool* tool, bool setActive)
    {
        // Adds the tool in the toolbar
        toolBarForTools->AddRadioTool(toolBarForTools->GetToolsCount(), wxEmptyString, wxBitmap(tool->getIcon()));
        wxAuiPaneInfo& pane = auiManager->GetPane(toolBarForTools);
        pane.MinSize(toolBarForTools->GetSize());
        auiManager->Update();

        // Adds the tool in the official list
        VisualizationFrame::addTool(tool, setActive);
    }

    void OCUFrame::setActiveTool(eu::nifti::ocu::tools::Tool* tool)
    {
        VisualizationFrame::setActiveTool(tool);

        // Selects the button in the toolbar
        int count = toolBarForTools->GetToolsCount();
        for (int i = 0; i < count; ++i)
        {
            if (tools.at(i) == activeTool)
            {
                toolBarForTools->ToggleTool(i, true);
                //                updateOCUinfoTool(i);  
                break;
            }
        }

        // Publishes a message to tell that the user activated a tool
        nifti_ocu_msgs::ToolActivation msg;
        msg.header.stamp = ros::Time::now();
        msg.tool = tool->getID();
        publisherToolActivation.publish(msg);
    }

    // In this thread it is safe to modify the GUI

    void OCUFrame::onUpdate()
    {
        assert(wxThread::IsMain());

        updateVizModeAndViews();

        pnlFilmstrips[0]->onUpdateUI();
        pnlFilmstrips[1]->onUpdateUI();
        pnlFilmstrips[2]->onUpdateUI();
    }

    void OCUFrame::updateVizModeAndViews()
    {
        // If the user clicked on a camera button in the GUI thread, process it now
        if (vizModeWasRecentlySelected == true)
        {
            vizModeWasRecentlySelected = false;
            setVizModeSynchronized(recentlySelectedMultiVizMode);
        }

        // If the user clicked on a camera button in the GUI thread, process it now
        if (viewTypeWasRecentlySelected == true)
        {
            viewTypeWasRecentlySelected = false;
            setViewTypeSynchronized(recentlySelectedViewType, recentlySelectedViewTypePanelNumber);
        }
    }

    void OCUFrame::customizeFilmstripPanelForViewTypes(eu::nifti::ocu::gui::FilmstripPanelForViewTypes* pnlMaps, eu::nifti::ocu::gui::FilmstripPanelForViewTypes* pnlCameras)
    {
        if (NIFTiConstants::getRobotType() == "UGV")
        {
            pnlCameras->addViewType(VIEW_TYPE_OMNI_CAM, "Cam", "360", "360º Camera");
            pnlCameras->addViewType(VIEW_TYPE_VIRTUAL_PTZ_CAM, "Cam", "Zoom", "Virtual Pan-Tilt-Zoom Camera");
            pnlCameras->addViewType(VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY, "Cam", "Zoom_Plus", "Virtual Pan-Tilt-Zoom Camera With Overlays");
            pnlCameras->addViewType(VIEW_TYPE_ARM_CAM, "Cam", "Arm", "Arm Camera");
        }
        else if (NIFTiConstants::getRobotType() == "UAV")
        {
            pnlCameras->addViewType(VIEW_TYPE_UAV_CAM_DOWN, "Cam", "UAV_Down", "UAV Down-Facing Camera");
            pnlCameras->addViewType(VIEW_TYPE_UAV_CAM_FRONT, "Cam", "UAV_Front", "UAV Front Tilt Camera");
        }
        else if (NIFTiConstants::getRobotType() == "P3AT")
        {
            pnlCameras->addViewType(VIEW_TYPE_KINECT_CAM, "Cam", "Kinect", "Kinect");
        }

        // There is also the old PTZ cam from the P3-AT

        pnlMaps->addViewType(VIEW_TYPE_MAP_2D, "Virtual_Scene", "Map_2D", "Map in 2D");
        pnlMaps->addViewType(VIEW_TYPE_CHASE_2D, "Virtual_Scene", "Chase_2D", "Centered on robot 2D");

        pnlMaps->addViewType(VIEW_TYPE_MAP_3D, "Virtual_Scene", "Map_3D", "Map in 3D");
        pnlMaps->addViewType(VIEW_TYPE_CHASE_3D, "Virtual_Scene", "Chase_3D", "Centered on robot 3D");

        bool showDebugCams = false;
        if (NIFTiROSUtil::getParam("showDebugCams", showDebugCams) && showDebugCams)
        {
            pnlMaps->addViewType(VIEW_TYPE_FIRST_PERSON, "Debug", "Robot", "View from the robot's camera");
        }
    }

    void OCUFrame::onToolClicked(wxCommandEvent& event)
    {
        //ROS_INFO("IN OCUFrame::onToolClicked");

        // Writes a warning message for the new tools (that are not yet implemented)
        //int diff = toolBarForTools->GetToolsCount() - event.GetId();
        //if (diff <= 2)
        if (event.GetId() == 666 || event.GetId() == 999) // These are tools that are not yet implemented
        {
            ROS_WARN("This tool is not yet implemented");
            setActiveTool(defaultTool);
        }
        else
        {
            setActiveTool(tools.at(event.GetId()));
        }

        //ROS_INFO("OUT OCUFrame::onToolClicked");
    }

    eu::nifti::ocu::MultiVizMode OCUFrame::getVizMode() const
    {
        return multiVizPanel->getMode();
    }

    void OCUFrame::setVizMode(eu::nifti::ocu::MultiVizMode mode)
    {
        vizModeWasRecentlySelected = true;
        recentlySelectedMultiVizMode = mode;
    }

    void OCUFrame::hideElementOfInterestPanel()
    {
            rightPanel->hideElementOfInterestPanel();
    }

    
    void OCUFrame::onSelectObjectOfInterest(const eu_nifti_env::ObjectOfInterest* ooi, bool selected)
    {
        if (selected)
        {
            if (ooi->element.status == eu_nifti_env::ElementOfInterest::STATUS_UNCONFIRMED)
            {
                // Shows the EOI Panel with info about the currently selected EOI
                const eu::nifti::ocu::display::ObjectOfInterestMarker* ooiMarker = (eu::nifti::ocu::display::ObjectOfInterestMarker*) selectionMgr->getSelectedMarker();
                rightPanel->showElementOfInterestPanel(ooi->element.uuid, ooiMarker->getCASTWorkingMemoryPointer());
            }
        }
        else
        {
            rightPanel->hideElementOfInterestPanel();
        }
    }

    void OCUFrame::onSelectLocationOfInterest(const eu_nifti_env::LocationOfInterest* loi, bool selected)
    {
        // For now, nothing to do
    }

    void OCUFrame::onSelectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture, bool selected)
    {
        if(selected)
        {
            if(rightPanel != NULL)  // Otherwise it's in UAV mode
            {
                rightPanel->showAnnotatedPictureInfo(picture);
            }
            threeHandlePanel->openPanel(3); // Opens the third panel (the one with the pictures)
        }
        else
        {
            if(rightPanel != NULL) // Otherwise it's in UAV mode
            {
                rightPanel->showAnnotatedPictureInfo(NULL);
            }
        }

        pnlInFieldPics->onSelectAnnotatedPicture(picture, selected);
    }

    void OCUFrame::setVizModeSynchronized(eu::nifti::ocu::MultiVizMode mode)
    {
        // Does nothing if trying to change to the same mode
        if (mode == multiVizPanel->getMode())
        {
            return;
        }

        //ROS_INFO("Current mode: %i", multiVizPanel->getMode());

        switch (mode)
        {
            case VIZ_MODE_SINGLE:
                //ROS_INFO("From ANY to SINGLE");

                // Removes the last possible 3 viz mgrs to keep only the first one (left, top, or top-left)
                removeView(1);
                removeView(2);
                removeView(3);

                multiVizPanel->setMode(VIZ_MODE_SINGLE, renderPanels.at(0));

                break;
            case VIZ_MODE_DUAL_VERTICAL:
                //ROS_INFO("From ANY to DUAL_VERTICAL");

                if (multiVizPanel->getMode() == VIZ_MODE_SINGLE)
                {
                    // Creates a new panel identical to the first one
                    renderPanels.at(1) = new RenderPanel(multiVizPanel, 1, sceneMgr, this);
                    renderPanels.at(1)->initialize();
                    renderPanels.at(1)->setAutoRender(false);
                    //vizMgrs.at(1) = vizMgrs.at(0)->duplicate(renderPanels.at(1));
                    vizMgrs.at(1) = new VisualizationManagerForBlankScreen(renderPanels.at(1), &(ocuMsgPubThread->guiStatus.viewParams.at(1)));
                    vizMgrs.at(1)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE
                }
                else if (multiVizPanel->getMode() == VIZ_MODE_QUAD)
                {
                    // Removes the last 2 viz mgrs to keep only the left column
                    removeView(1);
                    removeView(3);

                    // Moves the bottom left view to the bottom (index changes from 2 to 1)
                    swapView(2, 1);
                }

                multiVizPanel->setMode(VIZ_MODE_DUAL_VERTICAL, renderPanels.at(0), renderPanels.at(1));

                break;
            case VIZ_MODE_DUAL_HORIZONTAL:
                //ROS_INFO("From ANY to DUAL_HORIZONTAL");

                if (multiVizPanel->getMode() == VIZ_MODE_SINGLE)
                {
                    // Creates a new view identical to the first one
                    renderPanels.at(1) = new RenderPanel(multiVizPanel, 1, sceneMgr, this);
                    renderPanels.at(1)->initialize();
                    renderPanels.at(1)->setAutoRender(false);
                    //vizMgrs.at(1) = vizMgrs.at(0)->duplicate(renderPanels.at(1));
                    vizMgrs.at(1) = new VisualizationManagerForBlankScreen(renderPanels.at(1), &(ocuMsgPubThread->guiStatus.viewParams.at(1)));
                    vizMgrs.at(1)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE
                }
                else if (multiVizPanel->getMode() == VIZ_MODE_QUAD)
                {
                    // Removes the last 2 viz mgrs to keep only the top row
                    removeView(2);
                    removeView(3);
                }

                multiVizPanel->setMode(VIZ_MODE_DUAL_HORIZONTAL, renderPanels.at(0), renderPanels.at(1));
                break;
            case VIZ_MODE_QUAD:
                //ROS_INFO("From ANY to QUAD");

                if (multiVizPanel->getMode() == VIZ_MODE_SINGLE)
                {
                    // Creates 3 new views identical to the first one
                    renderPanels.at(1) = new RenderPanel(multiVizPanel, 1, sceneMgr, this);
                    renderPanels.at(1)->initialize();
                    renderPanels.at(1)->setAutoRender(false);
                    //vizMgrs.at(1) = vizMgrs.at(0)->duplicate(renderPanels.at(1));
                    vizMgrs.at(1) = new VisualizationManagerForBlankScreen(renderPanels.at(1), &(ocuMsgPubThread->guiStatus.viewParams.at(1)));
                    vizMgrs.at(1)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE

                    renderPanels.at(2) = new RenderPanel(multiVizPanel, 2, sceneMgr, this);
                    renderPanels.at(2)->initialize();
                    renderPanels.at(2)->setAutoRender(false);
                    //vizMgrs.at(2) = vizMgrs.at(0)->duplicate(renderPanels.at(2));
                    vizMgrs.at(2) = new VisualizationManagerForBlankScreen(renderPanels.at(2), &(ocuMsgPubThread->guiStatus.viewParams.at(2)));
                    vizMgrs.at(2)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE

                    renderPanels.at(3) = new RenderPanel(multiVizPanel, 3, sceneMgr, this);
                    renderPanels.at(3)->initialize();
                    renderPanels.at(3)->setAutoRender(false);
                    //vizMgrs.at(3) = vizMgrs.at(0)->duplicate(renderPanels.at(3));
                    vizMgrs.at(3) = new VisualizationManagerForBlankScreen(renderPanels.at(3), &(ocuMsgPubThread->guiStatus.viewParams.at(3)));
                    vizMgrs.at(3)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE
                }
                else if (multiVizPanel->getMode() == VIZ_MODE_DUAL_VERTICAL)
                {
                    // Moves the bottom view to the bottom left (index changes from 1 to 2)
                    swapView(1, 2);
                    //                    vizMgrs.at(2) = vizMgrs.at(1);
                    //                    renderPanels.at(2) = renderPanels.at(1);
                    //                    renderPanels.at(2)->setPanelNumber(2);

                    // Creates 2 new views identical to the first two
                    renderPanels.at(1) = new RenderPanel(multiVizPanel, 1, sceneMgr, this);
                    renderPanels.at(1)->initialize();
                    renderPanels.at(1)->setAutoRender(false);
                    //vizMgrs.at(1) = vizMgrs.at(0)->duplicate(renderPanels.at(1));
                    vizMgrs.at(1) = new VisualizationManagerForBlankScreen(renderPanels.at(1), &(ocuMsgPubThread->guiStatus.viewParams.at(1)));
                    vizMgrs.at(1)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE

                    renderPanels.at(3) = new RenderPanel(multiVizPanel, 3, sceneMgr, this);
                    renderPanels.at(3)->initialize();
                    renderPanels.at(3)->setAutoRender(false);
                    //vizMgrs.at(3) = vizMgrs.at(2)->duplicate(renderPanels.at(3));
                    vizMgrs.at(3) = new VisualizationManagerForBlankScreen(renderPanels.at(3), &(ocuMsgPubThread->guiStatus.viewParams.at(3)));
                    vizMgrs.at(3)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE
                }
                else if (multiVizPanel->getMode() == VIZ_MODE_DUAL_HORIZONTAL)
                {
                    // Creates 2 new views identical to the first two
                    renderPanels.at(2) = new RenderPanel(multiVizPanel, 2, sceneMgr, this);
                    renderPanels.at(2)->initialize();
                    renderPanels.at(2)->setAutoRender(false);
                    //vizMgrs.at(2) = vizMgrs.at(0)->duplicate(renderPanels.at(2));
                    vizMgrs.at(2) = new VisualizationManagerForBlankScreen(renderPanels.at(2), &(ocuMsgPubThread->guiStatus.viewParams.at(2)));
                    vizMgrs.at(2)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE

                    renderPanels.at(3) = new RenderPanel(multiVizPanel, 3, sceneMgr, this);
                    renderPanels.at(3)->initialize();
                    renderPanels.at(3)->setAutoRender(false);
                    //vizMgrs.at(3) = vizMgrs.at(1)->duplicate(renderPanels.at(3));
                    vizMgrs.at(3) = new VisualizationManagerForBlankScreen(renderPanels.at(3), &(ocuMsgPubThread->guiStatus.viewParams.at(3)));
                    vizMgrs.at(3)->initialize(); // WARNING THIS MIGHT BE TOO SOON BECAUSE IT'S BEFORE MULTIVIZPANEL->SETMODE
                }

                multiVizPanel->setMode(VIZ_MODE_QUAD, renderPanels.at(0), renderPanels.at(1), renderPanels.at(2), renderPanels.at(3));
                break;
        }

        //        // This does not work and I don't know why
        //        // Re-renders all Viz Mgrs just in case (otherwise some stay completely or half-black)
        //        for (int i = 0; i < MAX_NUM_VIEWS && vizMgrs.at(i) != NULL; i++)
        //        {
        //            renderPanels.at(i)->Layout();
        //            vizMgrs.at(i)->forceReRender();
        //        }

        adjustMouseCursors();

        // Updates all views in the info published about the OCU
        ocuMsgPubThread->guiStatus.vizMode = mode;
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (vizMgrs.at(i) == NULL || vizMgrs.at(i)->getViewport()->getCamera() == NULL) break; // Todo Check if this is correct

            updateOCUinfoViewSize(i);
            updateOCUinfoViewType(i);
        }

        // Publishes a message to tell that the user used the widget
        nifti_ocu_msgs::ViewControl msgUserAction;
        msgUserAction.stamp = ros::Time::now();
        msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
        msgUserAction.action = nifti_ocu_msgs::ViewControl::CHANGE_VIEW_MODE;
        publisherViewControl.publish(msgUserAction);

    }

    eu::nifti::ocu::ViewType OCUFrame::getViewType(u_int vizPanel)
    {
        switch (multiVizPanel->getMode())
        {
            case VIZ_MODE_SINGLE:
                assert(vizPanel < 1);
                break;
            case VIZ_MODE_DUAL_VERTICAL:
            case VIZ_MODE_DUAL_HORIZONTAL:
                assert(vizPanel < 2);
                break;
            case VIZ_MODE_QUAD:
                assert(vizPanel < 4);
        }

        return vizMgrs.at(vizPanel)->getViewType();
    }

    void OCUFrame::setViewType(eu::nifti::ocu::ViewType viewType, u_int vizPanel)
    {
        // Just saves the selection so that it gets processed with the update thread
        viewTypeWasRecentlySelected = true;
        recentlySelectedViewType = viewType;
        recentlySelectedViewTypePanelNumber = vizPanel;
    }

    void OCUFrame::setViewTypeSynchronized(eu::nifti::ocu::ViewType viewType, u_int vizPanel)
    {
        //ROS_INFO("IN void OCUFrame::setViewTypeSynchronized");

        assert(viewType != VIEW_TYPE_UNINITIALIZED);

        switch (multiVizPanel->getMode())
        {
            case VIZ_MODE_SINGLE:
                assert(vizPanel < 1);
                break;
            case VIZ_MODE_DUAL_VERTICAL:
            case VIZ_MODE_DUAL_HORIZONTAL:
                assert(vizPanel < 2);
                break;
            case VIZ_MODE_QUAD:
                assert(vizPanel < 4);
        }

        eu::nifti::ocu::ViewType currentViewType = vizMgrs.at(vizPanel)->getViewType();
        eu::nifti::ocu::ViewCategory currentViewCategory = NIFTiViewsUtil::getViewCategory(currentViewType);
        eu::nifti::ocu::ViewCategory viewCategory = NIFTiViewsUtil::getViewCategory(viewType);

        //ROS_INFO("setViewTypeSynchronized for panel #%i from %i to %i", vizPanel, currentViewType, viewType);

        /////////////////////////////////
        // 1) Switch Viz Mgr if needed //
        /////////////////////////////////


        // If the user changes from a cam to a virtual scene or vice-versa (or if the panel is still blank)
        if (currentViewCategory != viewCategory)
        {
            //ROS_INFO_STREAM("Must change Viz Mgr. Category from " << currentViewCategory << " to " << viewCategory);

            delete vizMgrs.at(vizPanel);

            switch (viewCategory)
            {
                case eu::nifti::ocu::VIEW_CATEGORY_CAMERA:
                    vizMgrs.at(vizPanel) = new VisualizationManagerForCamera(sceneMgr, new rviz::FrameTransformer(), selectionMgr, renderPanels.at(vizPanel), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(vizPanel)));
                    break;
                case eu::nifti::ocu::VIEW_CATEGORY_VIRTUAL_SCENE:
                    vizMgrs.at(vizPanel) = new VisualizationManagerFor3DScene(sceneMgr, virtualSceneManager->getFrameTransformer(), selectionMgr, renderPanels.at(vizPanel), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(vizPanel)));
                    break;
                case eu::nifti::ocu::VIEW_CATEGORY_PHOTO:
                    vizMgrs.at(vizPanel) = new VisualizationManagerForPhoto(sceneMgr, virtualSceneManager->getFrameTransformer(), selectionMgr, renderPanels.at(vizPanel), &threaded_queue_, &(ocuMsgPubThread->guiStatus.viewParams.at(vizPanel)));
                    break;
                default: // It cannot be anything else (verified by assertion above)
                    assert(false);
            }

            vizMgrs.at(vizPanel)->initialize();
        }
        assert(vizMgrs.at(vizPanel) != NULL);



        //////////////////////////
        // 2) Set new view type //
        //////////////////////////


        switch (viewCategory)
        {
            case eu::nifti::ocu::VIEW_CATEGORY_CAMERA:
                try
                {
                    vizMgrs.at(vizPanel)->setViewType(viewType);
                }
                catch (image_transport::TransportLoadException& ex)
                {
                    std::cerr << "Visualization Panel #" << vizPanel << " will not display the video feed" << std::endl;
                    // Continue with the rest, and leave the viz panel with the image "No Image"
                }
                break;
            case eu::nifti::ocu::VIEW_CATEGORY_VIRTUAL_SCENE:
                vizMgrs.at(vizPanel)->setViewType(viewType);
                break;
            case eu::nifti::ocu::VIEW_CATEGORY_PHOTO:
            {
                VisualizationManagerForPhoto* vm = (VisualizationManagerForPhoto*) vizMgrs.at(vizPanel);
                vm->setPhoto(recentlySelectedAnnotatedPicture);
            }
                break;
            default: // It cannot be anything else (verified by assertion above)
                assert(false);
        }
        assert(vizMgrs.at(vizPanel)->getViewType() == viewType);

        adjustMouseCursors();

        {
            // Updates the info published about the OCU       
            updateOCUinfoViewType(vizPanel);

            // Publishes a message to tell that the user used the widget
            nifti_ocu_msgs::ViewControl msgUserAction;
            msgUserAction.stamp = ros::Time::now();
            msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
            msgUserAction.action = nifti_ocu_msgs::ViewControl::CHANGE_VIEW_TYPE;
            publisherViewControl.publish(msgUserAction);
        }

        //ROS_INFO("OUT setViewTypeSynchronized");
    }

    void OCUFrame::swapView(uint32_t indexFrom, uint32_t indexTo)
    {
        //ROS_INFO("IN void OCUFrame::swapView(uint32_t indexFrom, uint32_t indexTo) %i %i", indexFrom, indexTo);

        // Moves the a view to another position when reducing the total number of views
        vizMgrs.at(indexTo) = vizMgrs.at(indexFrom);
        vizMgrs.at(indexFrom) = NULL;
        renderPanels.at(indexTo) = renderPanels.at(indexFrom);
        renderPanels.at(indexFrom) = NULL;
        renderPanels.at(indexTo)->setPanelNumber(indexTo);
    }

    void OCUFrame::removeView(uint32_t index)
    {
        //ROS_INFO("IN void OCUFrame::removeView(uint32_t index) %i", index);

        // Do nothing if this view is already empty
        if (vizMgrs.at(index) == NULL)
        {
            return;
        }

        assert(vizMgrs.at(index) != NULL);
        delete vizMgrs.at(index);
        vizMgrs.at(index) = NULL;


        // Destroys the window safely (instead of the delete operator)
        // Also removes the render panel from the multi viz panel
        assert(renderPanels.at(index) != NULL);
        renderPanels.at(index)->Destroy();
        renderPanels.at(index) = NULL;

        // Resets the information published for this view
        // This is not necessary, but helpful when examining the topic
        ocuMsgPubThread->guiStatus.views[index] = -1;
        ocuMsgPubThread->guiStatus.viewParams[index].projectionType = -1;
        ocuMsgPubThread->guiStatus.viewParams[index].cameraPose = geometry_msgs::Pose();
        ocuMsgPubThread->guiStatus.viewParams[index].fieldOfViewHorizontal = -1;
        ocuMsgPubThread->guiStatus.viewParams[index].fieldOfViewVertical = -1;
        ocuMsgPubThread->guiStatus.viewParams[index].viewWidth = -1;
        ocuMsgPubThread->guiStatus.viewParams[index].viewHeight = -1;
    }

    void OCUFrame::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt, int panelNum)
    {
        VisualizationFrame::handleMouseEvent(evt, panelNum);

        // Updates the info about the current cursor position
        ocuMsgPubThread->guiStatus.cursor_x = evt.evt.GetX();
        ocuMsgPubThread->guiStatus.cursor_y = evt.evt.GetY();
        ocuMsgPubThread->guiStatus.cursor_view = panelNum;
        //ocuMsgPubThread->guiStatus.cursor_time = t;


        if (evt.evt.LeftIsDown())
        {
            // If the left button is down and the mouse has moved, then the user is dragging
            if (evt.lastX != evt.evt.GetX() || evt.lastY != evt.evt.GetY())
            {
                draggingInProgress = true;
            }
        }
        else if (evt.evt.LeftUp())
        {
            // If the left button goes up when the user was dragging, then the drag is over
            if (draggingInProgress)
            {
                draggingInProgress = false;
            }// If the left button goes up when the user was not dragging, then this is a simple left click
            else
            {
                // Updates the info about the last click
                //                ocuMsgPubThread->guiStatus.last_click_x = evt.evt.GetX();
                //                ocuMsgPubThread->guiStatus.last_click_y = evt.evt.GetY();
                //                ocuMsgPubThread->guiStatus.last_click_view = panelNum;
                //                ocuMsgPubThread->guiStatus.last_click_time = t;
            }
        }

        // This does not work, unfortunately
        //        if(evt.evt.Leaving())
        //        {
        //            std::cout << "Leaving " << std::endl;
        //        }
    }

    void OCUFrame::handleSizeEvent(wxSizeEvent& evt, int panelNum)
    {
        updateOCUinfoViewSize(panelNum);

        VisualizationFrame::handleSizeEvent(evt, panelNum);
    }

    void OCUFrame::handleDragAndDropEvent(const wxString& text, int panelNum)
    {
        //ROS_INFO_STREAM("void OCUFrame::handleDragAndDropEvent(const wxString& text, int panelNum): " << text.mb_str());

        if (text.StartsWith(wxT("PIC: ")))
        {
            string filename(text.SubString(5, text.Length()).mb_str());

            try
            {
                recentlySelectedAnnotatedPicture = InFieldPicsManager::instance()->getPicture(filename);
            }
            catch (std::out_of_range& ex)
            {
                ROS_WARN_STREAM("No in-field picture with this name: " << filename);
                return;
            }
            assert(recentlySelectedAnnotatedPicture->completeFile.size() != 0);

            setViewType(eu::nifti::ocu::VIEW_TYPE_PHOTO, panelNum);
        }
        else
        {
            ViewType viewType = eu::nifti::ocu::ViewType(wxAtoi(text));

            if (NIFTiViewsUtil::viewTypeIsValid(viewType) == false)
            {
                ROS_WARN_STREAM("Cannot handle this view type: " << text.mb_str());
                return;
            }

            setViewType(viewType, panelNum);
        }

    }

    //    void OCUFrame::updateOCUinfoTool(int toolNum)
    //    {
    //        ocuMsgPubThread->guiStatus.currentTool = toolNum;
    //    }

    void OCUFrame::updateOCUinfoViewSize(int panelNum)
    {
        assert(vizMgrs.at(panelNum) != NULL);

        vizMgrs.at(panelNum)->updateOCUInfoViewSize();
    }

    void OCUFrame::updateOCUinfoViewType(int panelNum)
    {
        assert(vizMgrs.at(panelNum) != NULL);

        ocuMsgPubThread->guiStatus.views.at(panelNum) = vizMgrs.at(panelNum)->getViewType();
    }

    void OCUFrame::adjustMouseCursors()
    {
        for (int i = 0; i < MAX_NUM_VIEWS; i++)
        {
            if (renderPanels.at(i) == NULL) // Is this check still necessary?
            {
                break;
            }

            if (NIFTiViewsUtil::viewTypeIsCamera(vizMgrs.at(i)->getViewType()))
            {
                renderPanels.at(i)->SetCursor(activeTool->getCursorCamera());
            }
            else
            {
                renderPanels.at(i)->SetCursor(activeTool->getCursorVirtualScene());
            }

        }
    }

    void OCUFrame::forceRightPanelToExpand()
    {
        assert(rightPanel != NULL);

        // This is a hack, but since wxWidgets is so archaic, I have to do this manually
        // Ensures that the right panel expand to be as high as possible

        const int WX_FRAME_THICKNESS = 20; // Just an approximation
        int heightAvailableForRightPanel = this->GetSize().GetHeight() - topPanel->GetSize().GetHeight() - WX_FRAME_THICKNESS;

        //ROS_INFO_STREAM("this->GetSize().GetHeight() " << this->GetSize().GetHeight());
        //ROS_INFO_STREAM("topPanel->GetSize().GetHeight() " << topPanel->GetSize().GetHeight());
        //ROS_INFO_STREAM("heightAvailableForRightPanel " << heightAvailableForRightPanel);

        rightPanel->SetMinSize(wxSize(-1, heightAvailableForRightPanel));

        this->Layout();

        //rightPanel->Layout();
        rightPanel->Refresh();
        //rightPanel->Update();        
    }

}
