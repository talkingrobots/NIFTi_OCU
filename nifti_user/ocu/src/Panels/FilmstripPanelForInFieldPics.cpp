// Benoit 2013-01-29

#include <boost/foreach.hpp>

#include <wx/bitmap.h>
#include <wx/dnd.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/tooltip.h>

#include <nifti_pics_server_util/AnnotatedPicUtil.h>

#include "AnnotatedPicUtil.h"
#include "PictureDisplayUtil.h"

#include "Selection/SelectionManager.h"

#include "Panels/FilmstripPanelForInFieldPics.h"

using namespace std;
using namespace eu::nifti::ocu;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                const wxColour BACKGROUND_COLOR(0, 0, 0); // Black
                const wxColour SELECTED_COLOR(0, 255, 255); // Cyan

                FilmstripPanelForInFieldPics::FilmstripPanelForInFieldPics(wxWindow *parentWindow, eu::nifti::ocu::selection::SelectionManager* selMgr)
                : FilmstripPanel(parentWindow)
                , selMgr(selMgr)
                , numFramesCreated(0)
                , disableAutoScrolling(false)
                , selectedPicturePanel(NULL)
                {
                    scWindow->SetBackgroundColour(BACKGROUND_COLOR); // scWindow from the parent class

                    InFieldPicsManager::instance()->addListener(this);
                }

                FilmstripPanelForInFieldPics::~FilmstripPanelForInFieldPics()
                {
                    InFieldPicsManager::instance()->removeListener(this);

                    boost::mutex::scoped_lock lock(picturesMutex);

                    BOOST_FOREACH(const EXIFReader_msgs::AnnotatedPicture* picture, pictures)
                    {
                        delete picture;
                    }
                }

                wxString FilmstripPanelForInFieldPics::getItemText(const u_int index) const
                {
                    // First gets the backPanel, then takes the only element on it, which is the StaticBitmap
                    return boxSizer->GetItem(index)->GetWindow()->GetChildren().front()->GetToolTip()->GetTip();
                }

                void FilmstripPanelForInFieldPics::insertNewFrame(const FrameToAdd& newFrame)
                {
                    wxPanel* backPanel = new wxPanel(scWindow, newFrame.frameID);

                    backPanel->SetBackgroundColour(BACKGROUND_COLOR);

                    wxStaticBitmap* staticBitmap = createStaticBitmap(newFrame, backPanel);

                    // Centers vertically
                    int looseVertical = THUMBNAIL_SIZE - staticBitmap->GetSize().GetHeight();
                    staticBitmap->SetPosition(wxPoint(BORDER_SIZE, looseVertical / 2));

                    // Sets the size to barely larger than the picture
                    backPanel->SetMinSize(wxSize(staticBitmap->GetSize().GetWidth() + 2 * BORDER_SIZE, THUMBNAIL_SIZE));

                    boxSizer->Insert(findInsertionIndex(newFrame.toolTipText), backPanel, 0, 0, 0);

                    staticBitmap->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(FilmstripPanelForInFieldPics::onMouseLeftDown), NULL, this);
                }

                void FilmstripPanelForInFieldPics::onInFieldPicReceived(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //cout << "IN FilmstripPanelForInFieldPics::onNewInFieldPicReceived" << endl;

                    assert(picture->completeFile.size() != 0);

                    {
                        boost::mutex::scoped_lock lock(picturesMutex);

                        // Stores the picture in an ordered list to retrieve it easily when the user clicks on the thumbnail
                        pictures.push_back(picture);
                    }

                    wxImage* img = AnnotatedPicUtil::createWxImage(picture); // This can be slow

                    // Makes the image a bit smaller so that we see the background panel and we can highlight it
                    // This resizes the image, so it may be slow
                    eu::nifti::gui::PictureDisplayUtil::shrink(img, THUMBNAIL_SIZE - 2 * BORDER_SIZE, THUMBNAIL_SIZE - 2 * BORDER_SIZE);

                    addFrame(numFramesCreated++, img, picture->filename);

                    delete img;

                    //cout << "OUT FilmstripPanelForInFieldPics::onNewInFieldPicReceived" << endl;
                }

                void FilmstripPanelForInFieldPics::onNewInFieldPicTaken(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //cout << "FilmstripPanelForInFieldPics::onNewInFieldPicTaken" << endl;
                    // For now, do nothing. Wait until the full picture comes
                }

                void FilmstripPanelForInFieldPics::onSelectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture, bool selected)
                {


                    if (selected)
                    {
                        int index = findIndex(picture->filename);

                        assert(index != -1);

                        if (disableAutoScrolling == true)
                        {
                            disableAutoScrolling = false;
                        }
                        else
                        {
                            //cout << "Focus on pic at index " << index << " (" << picture->filename << ")" << endl;

                            int zero = 0;
                            int positionItem = boxSizer->GetItem(index)->GetPosition().x;
                            int positionFirstItem = boxSizer->GetItem(zero)->GetPosition().x; // This is because the position of the first item is not always zero, and it changes while the OCU runs. Stupid wxWdigets.
                            int positionRelative = positionItem - positionFirstItem;

                            //cout << "Absolute position: " << positionItem << " pixels within the BoxSizer" << endl;
                            //cout << "Relative position: " << positionRelative << " pixels within the BoxSizer" << endl;
                            //cout << "The center of the pic is thus at " << positionRelative + THUMBNAIL_SIZE / 2 << " pixels within the BoxSizer" << endl;
                            //cout << "scWindow->GetSize().GetWidth() is " << scWindow->GetSize().GetWidth() << " pixels" << endl;

                            int positionGoal = positionRelative + THUMBNAIL_SIZE / 2 - scWindow->GetSize().GetWidth() / 2;

                            //cout << "positionGoal is " << positionGoal << " pixels within the BoxSizer" << endl;
                            //cout << "positionGoal is " << (positionGoal / SCROLLBAR_UNIT_SIZE) << " scrolling units within the scrolled window" << endl;

                            scWindow->Scroll(positionGoal / SCROLLBAR_UNIT_SIZE, -1);
                        }

                        selectedPicturePanel = boxSizer->GetItem(index)->GetWindow();
                        selectedPicturePanel->SetBackgroundColour(SELECTED_COLOR);
                    }
                    else // Deselect
                    {
                        selectedPicturePanel->SetBackgroundColour(BACKGROUND_COLOR);
                        selectedPicturePanel = NULL;
                    }
                }

                void FilmstripPanelForInFieldPics::onMouseLeftDown(wxMouseEvent& event)
                {
                    const AnnotatedPicture* pic;
                    {
                        boost::mutex::scoped_lock lock(picturesMutex);

                        pic = pictures.at(event.GetId());
                    }

                    // Debugging only
                        //eu::nifti::misc::nifti_pics_server_util::AnnotatedPicUtil::printOutAnnotatedPictureDetails(pic);

                    disableAutoScrolling = true;

                    selMgr->selectAnnotatedPicture(pic);

                    std::stringstream filenameStream;
                    filenameStream << "PIC: " << pic->filename;

                    wxTextDataObject dragData(wxString(filenameStream.str().c_str(), wxConvUTF8));
                    wxDropSource dragSource(this);
                    dragSource.SetData(dragData);
                    dragSource.DoDragDrop(true);
                }

            } // End class
        }
    }


}
