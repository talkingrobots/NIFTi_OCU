// Benoit 2011-09-14
// Re-done 2013-01-29

#ifndef EU_NIFTI_OCU_GUI_FILMSTRIP_PANEL_H
#define EU_NIFTI_OCU_GUI_FILMSTRIP_PANEL_H

#include <vector>

#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/panel.h>

#include "NIFTiViewsUtil.h"

class wxBoxSizer;
class wxScrolledWindow;
class wxStaticBitmap;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                /**
                 * Panel that displays image frames like a filmstrip
                 */
                class FilmstripPanel : public wxPanel
                {
                public:

                    FilmstripPanel(wxWindow *parentWindow);
                    virtual ~FilmstripPanel();
                    
                    // Gives the information for a frame to be added (by the GUI thread at its next turn)
                    void addFrame(const wxWindowID frameID, wxImage* image, const std::string& toolTipText);
                    void addFrame(const wxWindowID frameID, const wxBitmap* image, const std::string& toolTipText);
                    
                    u_int findIndex(const std::string& itemText) const;
                    u_int findInsertionIndex(const std::string& itemText) const;
                    
                    virtual wxString getItemText(const u_int index) const = 0;
                    
                    // Can be called only from the wxWdigets GUI thread
                    void onUpdateUI();
                    
                    static const int THUMBNAIL_SIZE;
                    static const int SCROLLBAR_UNIT_SIZE;
                    static const int BORDER_SIZE;

                protected:
                    
                    class FrameToAdd;
                    
                    static wxStaticBitmap* createStaticBitmap(const FrameToAdd& frameToAdd, wxWindow* parent);
                    
                    virtual void insertNewFrame(const FrameToAdd& newFrame) = 0;
                    
                    wxScrolledWindow* scWindow;
                    wxBoxSizer* boxSizer;
                    
                    class FrameToAdd
                    {
                    public:
                        
                        FrameToAdd(const wxWindowID& frameID, const wxImage& img, const std::string& toolTipText)
                        : frameID(frameID)
                        , img(img)
                        , toolTipText(toolTipText)
                        {
                            
                        }
                        
                        FrameToAdd(const wxWindowID& frameID, const wxBitmap& bmp, const std::string& toolTipText)
                        : frameID(frameID)
                        , bmp(bmp)
                        , toolTipText(toolTipText)
                        {
                            
                        }
                                               
                        wxWindowID frameID;
                        wxImage img;
                        wxBitmap bmp;
                        std::string toolTipText;
                    };
                    
                    std::vector<FrameToAdd> framesToAdd;

                    boost::mutex framesToAddMutex;                    
                };

            }
        }
    }
}

#endif /* EU_NIFTI_PICVIEW_GUI_FILMSTRIP_PANEL_H */