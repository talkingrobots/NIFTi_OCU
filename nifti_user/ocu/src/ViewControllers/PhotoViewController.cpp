// Benoit 2013-04-02

#include "Displays/PhotoDisplay.h"

#include "Panels/MultiVizMouseEvent.h"

#include "ViewControllers/PhotoViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                const float PhotoViewController::ZOOM_SPEED = 0.002; // How fast the camera zooms with one move of the mouse

                PhotoViewController::PhotoViewController(eu::nifti::ocu::display::PhotoDisplay* display)
                : BlankViewController()
                , display(display)
                {
                }

                PhotoViewController::~PhotoViewController()
                {
                }

                void PhotoViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    if (evt.evt.Dragging())
                    {
                        int32_t diff_x = evt.evt.GetX() - evt.lastX;
                        int32_t diff_y = evt.evt.GetY() - evt.lastY;

                        if (evt.evt.LeftIsDown())
                        {
                            display->move(diff_x, diff_y);
                        }

                    }

                    if (evt.evt.GetWheelRotation() != 0)
                    {
                        int diff = evt.evt.GetWheelRotation();
                        display->zoom(1 + diff * ZOOM_SPEED);
                    }
                }
                
                double PhotoViewController::getFieldOfViewHorizontal() const
                {
                    return 0;
                }
                
                double PhotoViewController::getFieldOfViewVertical() const
                {
                    return 0;
                }
                
                void PhotoViewController::resetView()
                {
                    display->reset();
                }

          
            }
        }
    }
}
