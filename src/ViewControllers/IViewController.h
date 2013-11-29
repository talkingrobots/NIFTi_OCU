// Benoit 2011-11-17

#ifndef EU_NIFTI_OCU_VIEW_I_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_I_VIEW_CONTROLLER_H

#include <string>
#include <iostream>

namespace Ogre
{
    class Quaternion;
    class Vector3;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {
                class MultiVizKeyEvent;
                class MultiVizMouseEvent;
            }

            namespace view
            {

                class IViewController
                {
                public:
                    virtual ~IViewController() {};

                    virtual void activate() = 0;
                    virtual void deactivate() = 0;
                    virtual void update(float dt, float ros_dt) = 0;
                    virtual const std::string& getReferenceFrameName() const = 0;

                    virtual void handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt) = 0;
                    virtual void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt) = 0;

                    virtual Ogre::Vector3 getPosition() const = 0;
                    virtual Ogre::Quaternion getOrientation() const = 0;
                    
                    virtual double getFieldOfViewHorizontal() const = 0;
                    virtual double getFieldOfViewVertical() const = 0;
                    
                    virtual void lookAt(const Ogre::Vector3& point) = 0;
                    virtual void resetView() = 0;

                    virtual std::string toString() const = 0;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_I_VIEW_CONTROLLER_H
