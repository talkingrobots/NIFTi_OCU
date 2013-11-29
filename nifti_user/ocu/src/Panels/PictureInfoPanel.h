// Benoit 2013-05-15

#ifndef EU_NIFTI_OCU_GUI_PICTURE_INFO_PANEL_H
#define EU_NIFTI_OCU_GUI_PICTURE_INFO_PANEL_H

#include <wx/panel.h>

#include <ros/publisher.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

class wxBoxSizer;
class wxStaticText;
class wxTextCtrl;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            namespace gui
            {

                /**
                 * Displays a picture
                 */
                class PictureInfoPanel : public wxPanel
                {
                public:

                    PictureInfoPanel(wxWindow* parent);

                    void showAnnotatedPictureInfo(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                protected:
                    
                    void onTextUpdated(wxCommandEvent& evt);
                    
                    wxBoxSizer* sizer;
                    
                    wxStaticText* annotationLabel;
                    wxTextCtrl* annotationBox;
                    
                    std::vector<std::string> metadata;
                    
                    ros::Publisher publisher;

                private:
                    void connectTextUpdateEventHandler(bool connected);
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_GUI_PICTURE_INFO_PANEL_H
