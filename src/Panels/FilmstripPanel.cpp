// Benoit 2011-09-14
// Re-done 2013-01-29

#include <boost/foreach.hpp>

#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/image.h>
#include <wx/mstream.h>
#include <wx/tooltip.h>

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"
#include "PictureDisplayUtil.h"
#include "PictureMetadataUtil.h"

#include "Panels/FilmstripPanel.h"

using namespace std;
using namespace eu::nifti::ocu;
namespace fs = boost::filesystem;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                // These are just values that "feel" good
                const int FilmstripPanel::THUMBNAIL_SIZE = 4 * NIFTiConstants::MIN_BUTTON_SIZE;
                const int FilmstripPanel::SCROLLBAR_UNIT_SIZE = THUMBNAIL_SIZE / 2;
                const int FilmstripPanel::BORDER_SIZE = 4;

                FilmstripPanel::FilmstripPanel(wxWindow *parentWindow)
                : wxPanel(parentWindow)
                {
                    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

                    scWindow = new wxScrolledWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
                    boxSizer = new wxBoxSizer(wxHORIZONTAL);
                    scWindow->SetSizer(boxSizer);

                    sizer->Add(scWindow, 1, wxALL, 0);
                    this->SetSizer(sizer);

                    // Ensures that the height always stays the same
                    boxSizer->SetMinSize(wxSize(-1, THUMBNAIL_SIZE));
                    this->SetMinSize(wxSize(-1, THUMBNAIL_SIZE));

                    this->Layout();
                }

                FilmstripPanel::~FilmstripPanel()
                {
                }

                /**
                 * This can be called from a worker thread
                 * @param frameID
                 * @param image
                 * @param toolTipText
                 */
                void FilmstripPanel::addFrame(const wxWindowID frameID, wxImage* image, const std::string& toolTipText)
                {
                    eu::nifti::gui::PictureDisplayUtil::shrink(image, THUMBNAIL_SIZE, THUMBNAIL_SIZE);

                    {
                        boost::mutex::scoped_lock lock(framesToAddMutex);
                        framesToAdd.push_back(FrameToAdd(frameID, *image, toolTipText));
                    }

                }

                /**
                 * This can be called from a worker thread
                 * @param frameID
                 * @param image
                 * @param toolTipText
                 */
                void FilmstripPanel::addFrame(const wxWindowID frameID, const wxBitmap* image, const std::string& toolTipText)
                {
                    const wxBitmap rescaledImage = eu::nifti::gui::PictureDisplayUtil::shrink(*image, THUMBNAIL_SIZE, THUMBNAIL_SIZE);

                    {
                        boost::mutex::scoped_lock lock(framesToAddMutex);
                        framesToAdd.push_back(FrameToAdd(frameID, rescaledImage, toolTipText));
                    }

                }

                /**
                 * This affects the GUI, so it must be called from the GUI thread
                 */
                void FilmstripPanel::onUpdateUI()
                {
                    //ROS_INFO_STREAM("IN void FilmstripPanel::onUpdateUI(). # pics to add to filmstrip: " << framesToAdd.size() );

                    assert(wxThread::IsMain());

                    boost::mutex::scoped_lock lock(framesToAddMutex);

                    BOOST_FOREACH(const FrameToAdd& frameToAdd, framesToAdd)
                    {
                        insertNewFrame(frameToAdd);

                        scWindow->SetScrollbars(SCROLLBAR_UNIT_SIZE, 0, boxSizer->GetMinSize().GetWidth() / SCROLLBAR_UNIT_SIZE, 0);

                        boxSizer->Layout();
                        this->Layout();
                    }

                    framesToAdd.clear();

                    //ROS_INFO_STREAM("OUT void FilmstripPanel::onUpdateUI(). # pics to add to filmstrip: " << framesToAdd.size() );
                }

                wxStaticBitmap* FilmstripPanel::createStaticBitmap(const FrameToAdd& frameToAdd, wxWindow* parent)
                {
                    wxStaticBitmap* staticBitmap;

                    if (frameToAdd.bmp.GetWidth() == -1 && frameToAdd.bmp.GetHeight() == -1)
                    {
                        //Need to convert the wxImage to a wxBitmap (can be done only in the GUI thread)
                        staticBitmap = new wxStaticBitmap(parent, frameToAdd.frameID, wxBitmap(frameToAdd.img));
                    }
                    else
                    {
                        staticBitmap = new wxStaticBitmap(parent, frameToAdd.frameID, frameToAdd.bmp);
                    }

                    staticBitmap->SetToolTip(wxString::FromAscii(frameToAdd.toolTipText.c_str()));

                    return staticBitmap;
                }

                u_int FilmstripPanel::findIndex(const std::string& itemText) const
                {
                    wxString wx_itemText(itemText.c_str(), wxConvUTF8);

                    u_int index;
                    for (index = 0; index < boxSizer->GetChildren().size(); index++)
                    {
                        if (getItemText(index) == wx_itemText)
                            return index;
                    }

                    return -1;
                }

                u_int FilmstripPanel::findInsertionIndex(const std::string& itemText) const
                {
                    wxString newItemText(itemText.c_str(), wxConvUTF8);

                    u_int index;
                    for (index = 0; index < boxSizer->GetChildren().size(); index++)
                    {
                        if (getItemText(index) > newItemText)
                        {
                            return index;
                        }
                    }

                    return -1;
                }



            } // End-class
        }
    }


}
