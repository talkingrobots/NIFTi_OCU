// Benoit 2013-01-29

#ifndef EU_NIFTI_OCU_GUI_FILMSTRIP_PANEL_FOR_IN_FIELD_PICS_H
#define EU_NIFTI_OCU_GUI_FILMSTRIP_PANEL_FOR_IN_FIELD_PICS_H

#include <vector>

#include "InFieldPicsManager.h"

#include "Panels/FilmstripPanel.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {
                class SelectionManager;
            }
            
            namespace gui
            {

                /**
                 * Panel that goes at the bottom of the screen and displays thumbnails of pictures coming from the
                 * InFieldPicsManager
                 */
                class FilmstripPanelForInFieldPics : public FilmstripPanel, InFieldPicsManager::Listener
                {
                public:

                    FilmstripPanelForInFieldPics(wxWindow *parentWindow, eu::nifti::ocu::selection::SelectionManager* selMgr);
                    virtual ~FilmstripPanelForInFieldPics();
                    
                    wxString getItemText(const u_int index) const;
                    
                    void onInFieldPicReceived(const EXIFReader_msgs::AnnotatedPicture* picture);
                    void onNewInFieldPicTaken(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                    void onSelectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture, bool selected);
                    
                protected:

                    void insertNewFrame(const FrameToAdd& newFrame);
                    
                    void onMouseLeftDown(wxMouseEvent& event);

                    eu::nifti::ocu::selection::SelectionManager* selMgr;

                    std::vector<const EXIFReader_msgs::AnnotatedPicture*> pictures; // To be accessed by index
                    
                    boost::mutex picturesMutex;

                    u_int numFramesCreated;
                    
                    bool disableAutoScrolling;
                    
                    wxWindow* selectedPicturePanel;
                };

            }
        }
    }
}

#endif /* EU_NIFTI_PICVIEW_GUI_FILMSTRIP_PANEL_H */