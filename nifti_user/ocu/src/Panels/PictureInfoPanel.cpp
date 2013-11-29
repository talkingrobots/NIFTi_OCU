// Benoit 2013-05-15

#include <sstream>

#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <ros/node_handle.h>
#include <ros/this_node.h>

#include <EXIFReader_msgs/Modification.h>

#include "NIFTiROSUtil.h"

#include "PictureInfoPanel.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                const std::string TOPIC_MODIFY = "/inFieldPicsServer/modify";

                PictureInfoPanel::PictureInfoPanel(wxWindow* parent)
                : wxPanel(parent, wxID_ANY)
                {
                    const int MIN_HEIGHT_TEXT_BOX = 160;

                    const wxColour COLOUR_BLACK(0, 0, 0);
                    const wxColour COLOUR_WHITE(255, 255, 255);


                    this->SetBackgroundColour(COLOUR_BLACK);
                    //this->SetBackgroundColour(wxColour(0, 255, 0)); // Just for debugging


                    sizer = new wxBoxSizer(wxVERTICAL);


                    // To do: figure out why the centering does not work
                    annotationLabel = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE | wxST_NO_AUTORESIZE); // This size ensures that the label takes all of the available space
                    //annotationLabel->setMinSize(wxSize(-1, 40));
                    annotationLabel->SetForegroundColour(COLOUR_WHITE); // Makes the text white
                    sizer->Add(annotationLabel, 0, wxNO_BORDER, 0);

                    annotationBox = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_PROCESS_TAB); // This size ensures that the label takes all of the available space
                    annotationBox->SetBackgroundColour(COLOUR_WHITE);
                    sizer->Add(annotationBox, 1, wxNO_BORDER | wxEXPAND);

                    // Ensures a minimum height for the textbox
                    sizer->SetMinSize(wxSize(-1, MIN_HEIGHT_TEXT_BOX));

                    sizer->SetSizeHints(this); // set size hints to honour minimum size
                    this->SetSizer(sizer);
                    sizer->Layout();


                    publisher = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<EXIFReader_msgs::Modification > (TOPIC_MODIFY, 100, false);


                    connectTextUpdateEventHandler(false);
                }

                void PictureInfoPanel::showAnnotatedPictureInfo(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //std::cout << "IN void PictureInfoPanel::showAnnotatedPictureInfo(const EXIFReader_msgs::AnnotatedPicture* picture, const std::vector<std::string>& metadata): " << picture->filename << std::endl;

                    connectTextUpdateEventHandler(false);
                    annotationLabel->SetLabel(wxEmptyString);
                    annotationBox->Clear();

                    if (picture != NULL)
                    {
                        annotationLabel->SetLabel(wxString(picture->filename.c_str(), wxConvUTF8));
                        annotationBox->AppendText(wxString(picture->annotation.c_str(), wxConvUTF8));
                    }

                    connectTextUpdateEventHandler(true);


                    // This is necessary, otherwise the text in the label does not appear
                    sizer->Layout();

                    //printf("OUT void PictureDisplayPanel::displayAnnotation(const std::vector<std::string>& metadata)\n");
                }

                void PictureInfoPanel::onTextUpdated(wxCommandEvent& evt)
                {
                    //std::cout << "Text updated @ " << ros::this_node::getName() << ": " << std::string(annotationBox->GetValue().mb_str()) << std::endl;

                    EXIFReader_msgs::Modification msg;
                    msg.filename = std::string(annotationLabel->GetLabel().mb_str());
                    msg.source = ros::this_node::getName(); 
                    msg.annotation = std::string(annotationBox->GetValue().mb_str());

                    publisher.publish(msg);
                }

                void PictureInfoPanel::connectTextUpdateEventHandler(bool connected)
                {
                    if (connected)
                        annotationBox->Connect(wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(PictureInfoPanel::onTextUpdated), NULL, this);
                    else
                        annotationBox->Disconnect(wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler(PictureInfoPanel::onTextUpdated), NULL, this);
                }
            }
        }
    }
}
