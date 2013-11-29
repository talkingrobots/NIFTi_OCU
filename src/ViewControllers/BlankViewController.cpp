// Benoit 2011-11-17

#include "ViewControllers/BlankViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                BlankViewController::BlankViewController()
                {
                }

                BlankViewController::~BlankViewController()
                {
                }

                void BlankViewController::activate()
                {
                }

                void BlankViewController::deactivate()
                {
                }

                void BlankViewController::update(float dt, float ros_dt)
                {

                }

                const std::string& BlankViewController::getReferenceFrameName() const
                {
                    return DEFAULT_STRING;
                }

                void BlankViewController::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
                {

                }

                void BlankViewController::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {

                }

                Ogre::Vector3 BlankViewController::getPosition() const
                {
                    return DEFAULT_POSITION;
                }

                Ogre::Quaternion BlankViewController::getOrientation() const
                {
                    return DEFAULT_ORIENTATION;
                }
                
                double BlankViewController::getFieldOfViewHorizontal() const
                {
                    return 0;
                }
                
                double BlankViewController::getFieldOfViewVertical() const
                {
                    return 0;
                }

                void BlankViewController::lookAt(const Ogre::Vector3& point)
                {
                }

                void BlankViewController::resetView()
                {
                }

                std::string BlankViewController::toString() const
                {
                    return DEFAULT_STRING;
                }

            }
        }
    }
}
