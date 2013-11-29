// Benoit 2013-01-25

#include <iostream>

#include <wx/aui/aui.h>

#include <nifti_pics_server_util/ExceptionWithString.h>

#include "NIFTiConstants.h"
#include "PullHandle.h"

#include "Panels/ThreeHandlePanel.h"

using namespace std;
using namespace eu::nifti::ocu::gui;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                ThreeHandlePanel::ThreeHandlePanel(wxWindow *parentWindow, wxAuiManager* auiManager)
                : wxPanel(parentWindow)
                , auiManager(auiManager)
                , panel1(NULL)
                , panel2(NULL)
                , panel3(NULL)
                , panelOpened(NULL)
                {
                    //this->SetBackgroundColour(wxColour(255, 0, 0)); // Temp for debugging

                    sizerVertical = new wxBoxSizer(wxVERTICAL);

                    try
                    {
                        std::string imagePathMaps = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/PullHandle_Maps.png";
                        std::string imagePathCams = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/PullHandle_Cams.png";
                        std::string imagePathPics = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/PullHandle_Pics.png";

                        handle1 = new PullHandle(this, this, imagePathMaps, imagePathMaps, PullHandle::UP, false);
                        handle2 = new PullHandle(this, this, imagePathCams, imagePathCams, PullHandle::UP, false);
                        handle3 = new PullHandle(this, this, imagePathPics, imagePathPics, PullHandle::UP, false);
                    }
                    catch (const eu::nifti::misc::ExceptionWithString& ex)
                    {
                        cerr << ex.what() << endl;
                        throw;
                    }

                    wxBoxSizer* sizerHorizontal = new wxBoxSizer(wxHORIZONTAL);
                    sizerHorizontal->Add(handle1, 0, wxALL);
                    sizerHorizontal->AddSpacer(4);
                    sizerHorizontal->Add(handle2, 0, wxALL);
                    sizerHorizontal->AddSpacer(4);
                    sizerHorizontal->Add(handle3, 0, wxALL);

                    sizerVertical->Add(sizerHorizontal, 0, wxALIGN_CENTER_HORIZONTAL | wxALL);

                    this->SetSizer(sizerVertical);

                    minSizeWithPanelsClosed = wxSize(3 * handle1->GetMinWidth(), handle1->GetMinHeight());
                    this->SetMinSize(minSizeWithPanelsClosed);

                    this->Layout();

                    handle1->setOpeneability(true);
                    handle2->setOpeneability(true);
                    handle3->setOpeneability(true);
                }

                // This affects the GUI, so it must be called from the GUI thread

                void ThreeHandlePanel::setPanels(wxPanel *panel1, wxPanel *panel2, wxPanel *panel3)
                {
                    assert(wxThread::IsMain());

                    this->panel1 = panel1;
                    this->panel2 = panel2;
                    this->panel3 = panel3;

                    sizerVertical->Add(panel1, 1, wxEXPAND);
                    sizerVertical->Add(panel2, 1, wxEXPAND);
                    sizerVertical->Add(panel3, 1, wxEXPAND);

                    sizerVertical->Hide(panel1);
                    sizerVertical->Hide(panel2);
                    sizerVertical->Hide(panel3);

                    sizerVertical->Layout();
                }

                void ThreeHandlePanel::onPullHandleClosed(PullHandle* handle)
                {
                    //cout << "void ThreeHandlePanel::onPullHandleClosed()" << endl;

                    assert(panelOpened != NULL);

                    closePanel();
                }

                void ThreeHandlePanel::onPullHandleOpened(PullHandle* handle)
                {
                    //cout << "void ThreeHandlePanel::onPullHandleOpened()" << endl;

                    // If there is already an opened panel, then do nothing
                    if (panelOpened != NULL)
                        return;

                    if (handle == handle1)
                        openPanel(panel1);
                    else if (handle == handle2)
                        openPanel(panel2);
                    else if (handle == handle3)
                        openPanel(panel3);
                }

                void ThreeHandlePanel::onPullHandleClicked(PullHandle* handle)
                {
                    //cout << "void ThreeHandlePanel::onPullHandleClicked()" << endl;

                    // If the panels are closed, then clicking does nothing
                    if (panelOpened == NULL) return;

                    if (handle == handle1)
                        openPanel(panel1);
                    else if (handle == handle2)
                        openPanel(panel2);
                    else if (handle == handle3)
                        openPanel(panel3);
                }

                /**
                 * This will affect the GUI, so it must be called from the GUI thread
                 * @param panelNumber
                 */
                void ThreeHandlePanel::openPanel(u_int panelNumber)
                {
                    if (panelNumber == 1)
                        openPanel(panel1);
                    else if (panelNumber == 2)
                        openPanel(panel2);
                    else if (panelNumber == 3)
                        openPanel(panel3);
                    else
                        throw "panelNumber out of bounds";
                }

                /**
                 * This affects the GUI, so it must be called from the GUI thread
                 * @param panelToOpen
                 */
                void ThreeHandlePanel::openPanel(wxPanel *panelToOpen)
                {
                    //cout << "IN void ThreeHandlePanel::openPanel(wxPanel *panelToOpen)" << endl;

                    assert(wxThread::IsMain());
                    assert(panelToOpen != NULL);

                    if (panelOpened == panelToOpen)
                        return;
                    
                    if (panelOpened == NULL)
                    {
                        handle1->setOpeneability(false);
                        handle2->setOpeneability(false);
                        handle3->setOpeneability(false);

                        handle1->setCloseability(true);
                        handle2->setCloseability(true);
                        handle3->setCloseability(true);
                    }
                    else
                    {
                        sizerVertical->Hide(panelOpened);
                    }
                    
                    panelOpened = panelToOpen;
                    
                    sizerVertical->Show(panelToOpen);
                    
                    adjustLayout();

                    //cout << "OUT void ThreeHandlePanel::openPanel(wxPanel *panelToOpen)" << endl;
                }

                /**
                 * This affects the GUI, so it must be called from the GUI thread
                 */
                void ThreeHandlePanel::closePanel()
                {
                    //cout << "void ThreeHandlePanel::closePanel()" << endl;

                    assert(wxThread::IsMain());

                    handle1->setOpeneability(true);
                    handle2->setOpeneability(true);
                    handle3->setOpeneability(true);

                    handle1->setCloseability(false);
                    handle2->setCloseability(false);
                    handle3->setCloseability(false);

                    sizerVertical->Hide(panelOpened);

                    panelOpened = NULL;
                    
                    adjustLayout();
                }

                /**
                 * This affects the GUI, so it must be called from the GUI thread
                 */
                void ThreeHandlePanel::adjustLayout()
                {
                    assert(wxThread::IsMain());

                    if (panelOpened == NULL)
                    {
                        this->SetMinSize(minSizeWithPanelsClosed);
                    }
                    else
                    {
                        this->SetMinSize(wxSize(minSizeWithPanelsClosed.GetWidth(), minSizeWithPanelsClosed.GetHeight() + panelOpened->GetMinSize().GetHeight()));
                    }

                    auiManager->GetPane(this).MinSize(this->GetMinWidth(), this->GetMinHeight());

                    this->GetParent()->Layout();

                    this->Refresh();
                }

            }
        }
    }


}
