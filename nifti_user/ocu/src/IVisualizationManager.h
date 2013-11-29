// Benoit September 2010


#ifndef EU_NIFTI_OCU_I_VISUALIZATION_MANAGER_H_
#define EU_NIFTI_OCU_I_VISUALIZATION_MANAGER_H_

#include <string>

#include <OGRE/OgreImage.h>

#include "nifti_ocu_msgs/NIFTiViewParams.h"

#include "NIFTiViewsUtil.h"

class wxIdleEvent;
class wxSizeEvent;

namespace Ogre
{
    class Viewport;
    class Vector3;
    class SceneManager;
}

namespace rviz
{
    class RenderPanel;
    class ViewController;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {
                class IViewController;
            }

            class IVisualizationManager
            {
            public:

                virtual ~IVisualizationManager() {};
                
                virtual void initialize() = 0;

                /**
                 * Gets the coordinate frame to which all fixed data is transformed
                 */
                virtual const std::string& getFixedFrame() const = 0;

                /**
                 * Gets the coordinate frame in which the data is displayed
                 */
                virtual const std::string& getTargetFrame() const = 0;
                
                virtual void resetTime() = 0;

                /**
                 * Returns the Ogre viewport, which links a camera and a rendering surface
                 * @return
                 */
                virtual Ogre::Viewport* getViewport() const = 0;

                /**
                 * Returns the object that controls how the scene is viewed
                 * @return
                 */
                virtual eu::nifti::ocu::view::IViewController* getViewController() const = 0;
                
                /**
                 * Returns the horizontal field of view in radians
                 * @return 
                 */
                virtual double getFieldOfViewHorizontal() const = 0;
                
                /**
                 * Returns the horizontal field of view in radians
                 * @return 
                 */
                virtual double getFieldOfViewVertical() const = 0;

                /**
                 * Ensures that Ogre's renderer is called only in a mutex block
                 */
                virtual void lockRender() = 0;

                /**
                 * Ensures that Ogre's renderer is called only in a mutex block
                 */
                virtual void unlockRender() = 0;

                /**
                 * Queues a render request.
                 */
                virtual void queueRender() = 0;

                /**
                 * Forces the display to render at the next call to update (for now, only used for cameras - not 3D scenes)
                 */
                virtual void forceReRender() = 0;

                virtual void update(float wall_dt, float ros_dt) = 0;

                virtual void onIdle(wxIdleEvent& event) = 0;

                /**
                 * Makes the view point to a specific location
                 */
                virtual void lookAt(const Ogre::Vector3& point) = 0;

                /**
                 * Updates the info about the OCU that is periodically published
                 */
                virtual void updateOCUInfoViewContent() = 0;

                /**
                 * Updates the info about the OCU that is periodically published
                 */
                virtual void updateOCUInfoViewSize() = 0;
                
                /**
                 * Handles the resize event created by the render panel
                 * @param evt
                 * @param panelNum
                 */
                virtual void handleSizeEvent(wxSizeEvent& evt) = 0;

                /**
                 * Returns the render panel
                 * @return 
                 */
                virtual const rviz::RenderPanel* getRenderPanel() const = 0;

                /**
                 * Returns the x,y,z coordinates associated with a x,y mouse position
                 * within a viewport
                 * @param mouseX
                 * @param mouseY
                 * @return 
                 */
                virtual Ogre::Vector3 getFixedFramePositionFromViewportPosition(int mouseX, int mouseY) const = 0;

                /**
                 * Gets the view type (2D Map, 3D first person, etc.)
                 * @return 
                 */
                virtual ViewType getViewType() const = 0;

                /**
                 * Sets the view type (2D Map, 3D first person, etc.)
                 * @return 
                 */
                virtual void setViewType(eu::nifti::ocu::ViewType viewType) = 0;

                virtual void changeViewParams(nifti_ocu_msgs::NIFTiViewParams* viewParams) = 0;
                
                virtual Ogre::Image getCurrentImage() const = 0;
                
                virtual std::string getCurrentImageEncoding() const = 0;
                
                /**
                 * Returns debugging information
                 * @return
                 */
                virtual std::string toString() const = 0;

            };

        }
    }
}

#endif /* EU_NIFTI_OCU_I_VISUALIZATION_MANAGER_H_ */