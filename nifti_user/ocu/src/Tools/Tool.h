// Benoit August 2010

#ifndef EU_NIFTI_OCU_TOOLS_TOOL_H
#define EU_NIFTI_OCU_TOOLS_TOOL_H

#include <string>

#include <wx/image.h>

#include <OGRE/OgreVector3.h>

#include "Panels/MultiVizMouseEvent.h"
#include "Panels/MultiVizKeyEvent.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                class Tool
                {
                public:
                    Tool(int8_t id, const std::string& name, const std::string& iconFileName);

                    virtual ~Tool()
                    {
                    }

                    int8_t getID() const;

                    const std::string& getName() const;
                    
                    const wxImage& getIcon() const;

                    virtual void activate()
                    {
                    };

                    virtual void deactivate()
                    {
                    };

                    virtual void update(float wall_dt, float ros_dt)
                    {
                    }

                    /**
                     * Returns true iff we must switch back to the default tool after using
                     * this tool
                     * @param evt
                     * @return 
                     */
                    virtual bool handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                    {
                        return false;
                    };

                    /**
                     * Returns true iff we must switch back to the default tool after using
                     * this tool
                     * @param evt
                     * @return 
                     */
                    virtual bool handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
                    {
                        return false;
                    };

                    const wxCursor& getCursorCamera() const;
                    const wxCursor& getCursorVirtualScene() const;

                protected:
                    
                    void setCursorCamera(std::string fileName, int hotSpotX, int hotSpotY);
                    void setCursorCamera(wxCursor* cursor);
                    
                    void setCursorVirtualScene(std::string fileName, int hotSpotX, int hotSpotY);
                    void setCursorVirtualScene(wxCursor* cursor);
                    

                    Ogre::Vector3 getOgrePositionFromMouseCoordinates(Ogre::Viewport* viewport, int mouse_x, int mouse_y) const;

                    int8_t id;
                    std::string name;
                    wxImage icon;

                    wxCursor cursorCamera, cursorVirtualScene;

                    static const int CURSOR_SIZE;
                    
                private:
                    
                    wxCursor loadCursor(std::string fileName, int hotSpotX, int hotSpotY);
                };

            }
        }
    }
}

#endif
