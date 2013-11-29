// Benoit 2011-09-14
// Re-done 2013-01-29

#include <wx/dnd.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/tooltip.h>

#include "IMultiVisualizationManager.h"

#include "NIFTiConstants.h"

#include "Panels/FilmstripPanelForViewTypes.h"

using namespace std;
using namespace eu::nifti::ocu;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                
                FilmstripPanelForViewTypes::FilmstripPanelForViewTypes(wxWindow *parentWindow, IMultiVisualizationManager* multiVizMgr)
                : FilmstripPanel(parentWindow)
                , multiVizMgr(multiVizMgr)
                {
                    // This gives just a bit of a contrast.
                    scWindow->SetBackgroundColour(wxColour(220, 220, 220)); // scWindow from the parent class
                }

                FilmstripPanelForViewTypes::~FilmstripPanelForViewTypes()
                {

                }

                wxString FilmstripPanelForViewTypes::getItemText(const u_int index) const
                {
                    return boxSizer->GetItem(index)->GetWindow()->GetToolTip()->GetTip();
                }

                void FilmstripPanelForViewTypes::insertNewFrame(const FrameToAdd& newFrame)
                {
                    wxStaticBitmap* staticBitmap = createStaticBitmap(newFrame, scWindow);

                    boxSizer->Insert(findInsertionIndex(newFrame.toolTipText), staticBitmap, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, BORDER_SIZE);

                    staticBitmap->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(FilmstripPanelForViewTypes::onMouseLeftDown), NULL, this);
                }

                void FilmstripPanelForViewTypes::addViewType(const wxWindowID buttonID, const string& type, const string& name, const string& toolTipText)
                {
                    string completeImagePath;
                    wxBitmap bitmapData;
                    bool success;

                    completeImagePath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + "/View_" + type + '_' + name + ".png";

                    // Loads the background image
                    success = bitmapData.LoadFile(wxString::FromAscii(completeImagePath.c_str()));
                    assert(success);

                    addFrame(buttonID, &bitmapData, toolTipText);
                }

                void FilmstripPanelForViewTypes::onMouseLeftDown(wxMouseEvent& event)
                {
                    //cout << "IN void ViewTypeButtonPanel::onLeftDown(wxMouseEvent& event)" << endl;

                    wxString stringForMsg;
                    stringForMsg << event.GetId();

                    wxTextDataObject dragData(stringForMsg);
                    wxDropSource dragSource(this);
                    dragSource.SetData(dragData);
                    dragSource.DoDragDrop(true);

                    /*
                    wxDragResult result = dragSource.DoDragDrop(true);

                    if (wxIsDragResultOk(result))
                    {
                        cout << "SUCCESS" << endl;
                    }
                    else
                    {
                        cout << "FAILURE" << endl;
                    }*/

                    //cout << "OUT void ViewTypeButtonPanel::onLeftDown(wxMouseEvent& event)" << endl;
                }

            }
        }
    }


}
