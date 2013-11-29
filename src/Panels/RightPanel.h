// Benoit 2011-08-30

#ifndef EU_NIFTI_OCU_GUI_RIGHT_PANEL_H_
#define EU_NIFTI_OCU_GUI_RIGHT_PANEL_H_

#include <wx/panel.h>
#include <wx/sizer.h>

#include <ros/publisher.h>

#include <eu_nifti_cast/WorkingMemoryPointer.h>

#include "IPullHandleEventHandler.h"

namespace eu
{
    namespace nifti
    {
        
        namespace ocu
        {
            class IMultiVisualizationManager;
            
            namespace gui
            {

                class ElementOfInterestPanel;
                class PictureInfoPanel;
                class PullHandle;
                
                /**
                 * Contains the panels on the right
                 */
                class RightPanel : public wxPanel, public IPullHandleEventHandler
                {
                public:
                    RightPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr);
                    
                    void showElementOfInterestPanel(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer);
                    void hideElementOfInterestPanel();
                    
                    void showAnnotatedPictureInfo(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                    void onPullHandleClosed(PullHandle* handle);
                    void onPullHandleOpened(PullHandle* handle);
                    void onPullHandleClicked(PullHandle* handle);
                    
                protected:
                    void publishUserAction(int actionID);

                    IMultiVisualizationManager* multiVizMgr;
                    
                    wxBoxSizer* sizer;
                    
                    // The publisher is owned here because creating and destroying publishers (directly in the EOIPanel) is not well supported by ROS
                    ros::Publisher publisherEOIConfirmation;
                    
                    eu::nifti::ocu::gui::ElementOfInterestPanel* eoiPanel;
                    
                    eu::nifti::ocu::gui::PictureInfoPanel* pictureInfoPanel;
                    
                    ros::Publisher publisherUserActions, publisherControlAutonomousFlippers;                    
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_GUI_RIGHT_PANEL_H_ */
