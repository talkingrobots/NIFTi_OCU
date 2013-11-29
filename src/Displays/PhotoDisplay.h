/// Benoit 2013-03

#ifndef EU_NIFTI_OCU_DISPLAY_PHOTO_DISPLAY_H
#define EU_NIFTI_OCU_DISPLAY_PHOTO_DISPLAY_H

#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTexture.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include "Displays/Display.h"

class wxSizeEvent;

namespace Ogre
{
    class SceneNode;
    class Rectangle2D;
    class RenderWindow;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                /**
                 * Allows the display of a video feed
                 */
                class PhotoDisplay : public rviz::Display, public Ogre::RenderTargetListener
                {
                public:
                    PhotoDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, Ogre::RenderWindow* renderWindow);
                    virtual ~PhotoDisplay();

                    void zoom(double zoom);
                    void move(int x, int y);

                    /**
                     * Forces the display to render at the next call to update
                     */
                    void forceRender();

                    // Overrides from Display
                    virtual void fixedFrameChanged();
                    virtual void update(float wall_dt, float ros_dt);
                    virtual void reset();
                    
                    void handleSizeEvent(wxSizeEvent& evt);
                    
                    virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
                    virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

                    void setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture);

                protected:
                    
                    virtual void onEnable();
                    virtual void onDisable();
                    
                    void adjustDisplayRatios();
                    void adjustPhoto();

                    void checkZoomLimits();
                    void checkDisplacementLimits();
                    
                    Ogre::RenderWindow* renderWindow;

                    Ogre::SceneNode* sceneNode;
                    Ogre::Rectangle2D* screenRect;
                    Ogre::MaterialPtr material;

                    double compressionRatioX, compressionRatioY;
                    double zoomMin;
                   
                    double zoomLevel; // Magnification level, maxed out at 1 for now
                    int displacement_X, displacement_Y; // Displacement from the center of the picture in real picture pixels. That new point is to be centered in the viewport

                    Ogre::TexturePtr texture;
                    
                    bool renderRequested;
                    
                    static Ogre::Image DEFAULT_IMAGE; // Contains the image that says "Waiting for Image"
                    
                };

            }
        }
    }
}

#endif
