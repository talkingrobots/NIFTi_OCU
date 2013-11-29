// Benoit 2013-01-28

#include <wx/statbmp.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include "IPullHandleEventHandler.h"

#include "PullHandle.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                const int MINIMAL_MOVEMENT = 4; // 4 pixels is approx. 1mm

                PullHandle::PullHandle(wxWindow* parent, IPullHandleEventHandler *handler, const std::string& imagePathClosed, const std::string& imagePathOpened, OpeningDirection openingDirection, bool startOpened)
                : wxPanel(parent, wxID_ANY)
                , handler(handler)
                , draggingHandle(false)
                , OPENING_DIRECTION(openingDirection)
                , canOpen(false)
                , canClose(false)
                {                    
                    bool success;

                    success = bmpClosed.LoadFile(wxString::FromAscii(imagePathClosed.c_str()));
                    if(!success)
                    {
                        throw eu::nifti::misc::ExceptionWithString("Failed to load file: " + imagePathClosed);
                    }

                    success = bmpOpened.LoadFile(wxString::FromAscii(imagePathOpened.c_str()));
                    if(!success)
                    {
                        throw eu::nifti::misc::ExceptionWithString("Failed to load file: " + imagePathOpened);
                    }

                    if(startOpened == true)
                    {
                        staticBitmap = new wxStaticBitmap(this, -1, bmpOpened);
                    }
                    else
                    {
                        staticBitmap = new wxStaticBitmap(this, -1, bmpClosed);
                    }

                    this->SetMinSize(wxSize(bmpClosed.GetWidth(), bmpClosed.GetHeight()));

//                    // Assuming bitmaps of same size
//                    // I just picked 1/4 because it seemed appropriate
//                    if(OPENING_DIRECTION == UP || OPENING_DIRECTION == DOWN)
//                        MIN_MOVEMENT = bmpClosed.GetHeight() / 4;
//                    else
//                        MIN_MOVEMENT = bmpClosed.GetWidth() / 4;
                    
                    staticBitmap->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(PullHandle::onMouseDown), NULL, this);
                    staticBitmap->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(PullHandle::onMouseUp), NULL, this);
                    this->Connect(wxEVT_LEAVE_WINDOW, wxMouseEventHandler(PullHandle::onMouseLeave), NULL, this);
                }

                bool PullHandle::getOpeneability() const
                {
                    return canOpen;
                }

                void PullHandle::setOpeneability(bool canOpen)
                {
                    this->canOpen = canOpen;
                }

                bool PullHandle::getCloseability() const
                {
                    return canClose;
                }

                void PullHandle::setCloseability(bool canClose)
                {
                    this->canClose = canClose;
                }

                void PullHandle::onMouseDown(wxMouseEvent& event)
                {
                    draggingHandle = true;
                    xOnMouseDown = event.GetX();
                    yOnMouseDown = event.GetY();

                    //ROS_INFO_STREAM("PullHandle::onMouseDown - xOnMouseDown: " << xOnMouseDown  << " yOnMouseDown: " << yOnMouseDown);
                }

                void PullHandle::onMouseUp(wxMouseEvent& event)
                {
                    draggingHandle = false;
                    
                    // Basically, if the MouseDown was at the same location (or within a small limit), consider it as a click
                    int diff_x = abs(event.GetX() - xOnMouseDown);
                    int diff_y = abs(event.GetY() - yOnMouseDown);
                    if( diff_x <= MINIMAL_MOVEMENT && diff_y <= MINIMAL_MOVEMENT )
                    {
                        //ROS_INFO_STREAM("Click triggered");
                        handler->onPullHandleClicked(this);
                    }
                    
                    //ROS_INFO_STREAM("PullHandle::onMouseUp - diff_x: " << diff_x  << " diff_y: " << diff_y);
                }

                void PullHandle::onMouseLeave(wxMouseEvent& event)
                {
                    //ROS_INFO("PullHandle::onMouseLeave");

                    // If the user is not currently dragging the handle, then do nothing
                    if (!draggingHandle) return;

                    draggingHandle = false;

                    int diff_x = event.GetX() - xOnMouseDown;
                    int diff_y = event.GetY() - yOnMouseDown;

                    //ROS_INFO_STREAM("PullHandle::onMouseLeave - diff_x: " << diff_x << " diff_y: " << diff_y);

                    switch (OPENING_DIRECTION)
                    {
                        case UP:
                            if (diff_y <= -MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToOpen();
                            }
                            else if (diff_y >= MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToClose();
                            }
                            break;
                        case DOWN:
                            if (diff_y >= MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToOpen();
                            }
                            else if (diff_y <= -MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToClose();
                            }
                            break;
                        case LEFT:
                            if (diff_x <= -MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToOpen();
                            }
                            else if (diff_x >= MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToClose();
                            }
                            break;
                        case RIGHT:
                            if (diff_x >= MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToOpen();
                            }
                            else if (diff_x <= -MINIMAL_MOVEMENT)
                            {
                                onPullHandleDraggedToClose();
                            }
                            break;
                    }
                }

                void PullHandle::onPullHandleDraggedToOpen()
                {
                    //ROS_INFO("PullHandle::onPullHandleDraggedToOpen");

                    if (!canOpen) return;

                    staticBitmap->SetBitmap(bmpOpened);
                    handler->onPullHandleOpened(this);
                }

                void PullHandle::onPullHandleDraggedToClose()
                {
                    //ROS_INFO("PullHandle::onPullHandleDraggedToClose");

                    if (!canClose) return;

                    staticBitmap->SetBitmap(bmpClosed);
                    handler->onPullHandleClosed(this);
                }

            }
        }
    }
}