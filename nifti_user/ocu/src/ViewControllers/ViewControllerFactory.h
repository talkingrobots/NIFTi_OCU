// Benoit September 2010

#ifndef NIFTI_VIEW_CONTROLLER_FACTORY_H
#define NIFTI_VIEW_CONTROLLER_FACTORY_H

#include "IVisualizationManager.h"
#include "NIFTiViewsUtil.h"

#include "Displays/Transformers/FrameTransformer.h"

namespace Ogre
{
    class SceneManager;
    class Camera;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                class ViewControllerFactory
                {
                public:

                    static IViewController* createViewController(eu::nifti::ocu::ViewType viewType, Ogre::SceneManager* sceneMgr, Ogre::Camera* camera, IVisualizationManager* vizMgr, rviz::FrameTransformer* frameTransformer);

                private:

                    ViewControllerFactory();

                };

            }
        }
    }
}

#endif // NIFTI_VIEW_CONTROLLER_FACTORY_H
