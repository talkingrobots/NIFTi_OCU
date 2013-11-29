// Benoit August 2010

#include "wx/image.h"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreViewport.h>

#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "Tools/Tool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {


                const int Tool::CURSOR_SIZE = 32;

                Tool::Tool(int8_t id, const std::string& name, const std::string& iconFileName)
                : id(id)
                , name(name)
                , cursorCamera(*wxSTANDARD_CURSOR) // This is not super efficient, because the data is copied, but it makes the code clearer
                , cursorVirtualScene(*wxSTANDARD_CURSOR)
                {
                    std::string fullPath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + iconFileName + ".png";
                    icon.LoadFile(wxString::FromAscii(fullPath.c_str()), wxBITMAP_TYPE_PNG);
                    
                    // With images of size 38, the buttons in the toolbar will be 48
                    if (icon.GetWidth() != 38 || icon.GetHeight() != 38)
                    {
                        icon = icon.Scale(38, 38, wxIMAGE_QUALITY_HIGH);
                    }
                }

                int8_t Tool::getID() const
                {
                    return id;
                }

                const std::string& Tool::getName() const
                {
                    return name;
                }
                
                const wxImage& Tool::getIcon() const
                {
                    return icon;
                }

                const wxCursor& Tool::getCursorCamera() const
                {
                    return cursorCamera;
                }

                const wxCursor& Tool::getCursorVirtualScene() const
                {
                    return cursorVirtualScene;
                }
                               
                void Tool::setCursorCamera(std::string fileName, int hotSpotX, int hotSpotY)
                {
                    cursorCamera = loadCursor(fileName, hotSpotX, hotSpotY);
                }
                
                void Tool::setCursorCamera(wxCursor* cursor)
                {
                    cursorCamera = *cursor;
                }

                void Tool::setCursorVirtualScene(std::string fileName, int hotSpotX, int hotSpotY)
                {
                    cursorVirtualScene = loadCursor(fileName, hotSpotX, hotSpotY);
                }
                
                void Tool::setCursorVirtualScene(wxCursor* cursor)
                {
                    cursorVirtualScene = *cursor;
                }
                
                wxCursor Tool::loadCursor(std::string fileName, int hotSpotX, int hotSpotY)
                {
                    std::string fullPath = NIFTiConstants::ROS_PACKAGE_PATH + NIFTiConstants::IMAGE_FOLDER_PATH + '/' + fileName + ".png";
                    wxImage img;
                    img.LoadFile(wxString::FromAscii(fullPath.c_str()), wxBITMAP_TYPE_PNG);

                    img.SetOption(wxIMAGE_OPTION_CUR_HOTSPOT_X, hotSpotX);
                    img.SetOption(wxIMAGE_OPTION_CUR_HOTSPOT_Y, hotSpotY);
                    
                    return wxCursor(img);
                }

                Ogre::Vector3 Tool::getOgrePositionFromMouseCoordinates(Ogre::Viewport* viewport, int mouse_x, int mouse_y) const
                {
                    int width = viewport->getActualWidth();
                    int height = viewport->getActualHeight();

                    Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay((float) mouse_x / (float) width, (float) mouse_y / (float) height);
                    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
                    std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
                    if (!intersection.first)
                    {
                        return Ogre::Vector3::ZERO;
                    }

                    return mouse_ray.getPoint(intersection.second);
                }
            }
        }
    }
}