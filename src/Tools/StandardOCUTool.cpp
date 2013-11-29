// Benoit Benoit 2012-05-22

#include <OGRE/OgreViewport.h>

#include <ros/node_handle.h>

#include <nifti_ocu_msgs/ViewControl.h>

#include "IVisualizationManager.h"
#include "NIFTiROSUtil.h"

#include "ViewControllers/ViewController.h"

#include "Tools/StandardOCUTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {
                const u_int StandardOCUTool::DELAY_LONG_CLICK = 800; 
                // The Apple recommendation for iOS is 500 ms, but I find it too short (just a feeling)
                // Nokia is 800 ms

                StandardOCUTool::StandardOCUTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl)
                : Tool(id, name, iconFileName)
                , publisherViewControl(publisherViewControl)
                , timerLongClick(this)
                , dragging(false)
                , xOnMouseDown(0)
                , yOnMouseDown(0)
                , currentViewController(NULL)
                {
                }

                void StandardOCUTool::deactivate()
                {
                    // Clean-up operations (just for safety)
                    
                    dragging = false;
                    currentViewController = NULL;
                    timerLongClick.Stop();
                }

                bool StandardOCUTool::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
                {
                    // The keys have no meaning for this tool, so just forward the events to the view manager
                    evt.vizMgr->getViewController()->handleKeyEvent(evt);
                    return false;
                }

                bool StandardOCUTool::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    currentViewController = evt.vizMgr->getViewController();                    
                    currentViewController->handleMouseEvent(evt);

                    // If the user just pushed the button down, then the selection process starts
                    if (evt.evt.LeftDown() || evt.evt.LeftDClick()) // wxWidgets sometimes sends DClick events instead of LeftDown
                    {
                        xOnMouseDown = evt.evt.GetX();
                        yOnMouseDown = evt.evt.GetY();
                        timeOnMouseDown = evt.evt.GetTimestamp();

                        // Timer to update the display every second
                        //timerUpdate = new wxTimer(this);
                        Connect(timerLongClick.GetId(), wxEVT_TIMER, wxTimerEventHandler(StandardOCUTool::onTimerLongClick));
                        timerLongClick.Start(DELAY_LONG_CLICK, true);
                    }
                    else if (evt.evt.LeftUp())
                    {
                        timerLongClick.Stop();

                        if (dragging == true)
                        {
                            // Publishes a message to tell that the user stopped moving the view
                            nifti_ocu_msgs::ViewControl msgUserAction;
                            msgUserAction.stamp = ros::Time::now();
                            msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                            msgUserAction.action = nifti_ocu_msgs::ViewControl::MOVE + nifti_ocu_msgs::ViewControl::STOP;
                            publisherViewControl->publish(msgUserAction);

                            dragging = false;
                        }
                        else
                        {
                            u_int duration = evt.evt.GetTimestamp() - timeOnMouseDown;
                            if (duration < DELAY_LONG_CLICK)
                            {
                                return onShortClick(evt);
                            }
                        }

                        currentViewController = NULL;
                    }
                    else if (evt.evt.Dragging())
                    {
                        if (dragging == false)
                        {
                            dragging = true;
                            timerLongClick.Stop(); // No long click possible anymore
                            
                            // Publishes a message to tell that the user started moving the view
                            nifti_ocu_msgs::ViewControl msgUserAction;
                            msgUserAction.stamp = ros::Time::now();
                            msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                            msgUserAction.action = nifti_ocu_msgs::ViewControl::MOVE + nifti_ocu_msgs::ViewControl::START;
                            publisherViewControl->publish(msgUserAction);
                        }
                    }
                    else if (evt.evt.GetWheelRotation() != 0)
                    {
                        // Publishes a message to tell that the user used the feature
                        nifti_ocu_msgs::ViewControl msgUserAction;
                        msgUserAction.stamp = ros::Time::now();
                        msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                        msgUserAction.action = nifti_ocu_msgs::ViewControl::ZOOM;
                        publisherViewControl->publish(msgUserAction);
                    }

                    return false;
                } // end method

                void StandardOCUTool::onTimerLongClick(wxTimerEvent& event)
                {
                    currentViewController->resetView();

                    // Publishes a message to tell that the user used the feature
                    nifti_ocu_msgs::ViewControl msgUserAction;
                    msgUserAction.stamp = ros::Time::now();
                    msgUserAction.userID = NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    msgUserAction.action = nifti_ocu_msgs::ViewControl::RESET;
                    publisherViewControl->publish(msgUserAction);
                }

            } // end class
        }
    }

}

