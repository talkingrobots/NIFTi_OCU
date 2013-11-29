// Benoit 2013-04-02

#ifndef NIFTI_PHOTO_VIEW_CONTROLLER_H
#define NIFTI_PHOTO_VIEW_CONTROLLER_H

#include "ViewControllers/BlankViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {
                class PhotoDisplay;
            }

            namespace view
            {

                class PhotoViewController : public BlankViewController
                {
                public:
                    PhotoViewController(eu::nifti::ocu::display::PhotoDisplay* display);
                    virtual ~PhotoViewController();

                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);
                    
                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;
                    
                    void resetView();
                    
                protected:
                    eu::nifti::ocu::display::PhotoDisplay* display;

                    static const float ZOOM_SPEED;
                };

            }
        }
    }
}

#endif // NIFTI_PHOTO_VIEW_CONTROLLER_H
