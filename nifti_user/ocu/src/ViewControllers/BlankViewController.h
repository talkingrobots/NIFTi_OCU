// Benoit 2011-11-17

#ifndef EU_NIFTI_OCU_VIEW_BLANK_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_BLANK_VIEW_CONTROLLER_H

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "ViewControllers/IViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                /**
                 * Most basic view controller: does nothing
                 */
                class BlankViewController : public IViewController
                {
                public:
                    BlankViewController();
                    virtual ~BlankViewController();

                    virtual void activate();
                    virtual void deactivate();
                    virtual void update(float dt, float ros_dt);
                    virtual const std::string& getReferenceFrameName() const;

                    virtual void handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt);
                    virtual void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    virtual Ogre::Vector3 getPosition() const;
                    virtual Ogre::Quaternion getOrientation() const;
                    
                    virtual double getFieldOfViewHorizontal() const;
                    virtual double getFieldOfViewVertical() const;
                    
                    virtual void lookAt(const Ogre::Vector3& point);
                    virtual void resetView();

                    virtual std::string toString() const;

                private:
                    const std::string DEFAULT_STRING;
                    const Ogre::Vector3 DEFAULT_POSITION;
                    const Ogre::Quaternion DEFAULT_ORIENTATION;
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_BLANK_VIEW_CONTROLLER_H
