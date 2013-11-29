// Benoit 2012-03-01

#ifndef EU_NIFTI_OCU_GUI_EOI_PANEL_H_
#define EU_NIFTI_OCU_GUI_EOI_PANEL_H_

#include <wx/panel.h>

#include <ros/publisher.h>

#include <eu_nifti_cast/WorkingMemoryPointer.h>

class wxButton;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class IMultiVisualizationManager;
            
            namespace gui
            {

                /**
                 * Displays buttons to confirm or reject an object detection. Eventually, also a text box to type in comments
                 */
                class ElementOfInterestPanel : public wxPanel
                {
                public:
                    ElementOfInterestPanel(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr, int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, ros::Publisher& publisherEOIConfirmation);

                protected:
                    void onBtnConfirmPressed(wxCommandEvent& evt);
                    void onBtnRejectPressed(wxCommandEvent& evt);

                    void publishConfirmationMessage(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, int8_t status);
                    
                    IMultiVisualizationManager* multiVizMgr;
                    
                    ros::Publisher& publisherEOIConfirmation;
                    
                    int uuid;
                    eu_nifti_cast::WorkingMemoryPointer castWorkingMemoryPointer;

                    static const char* CONFIRMATION_TOPIC;

                    static const int ID_BTN_CONFIRM;
                    static const int ID_BTN_REJECT;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_OCU_GUI_EOI_PANEL_H_ */