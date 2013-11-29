// Benoit July 2010

#include "ViewControllers/IViewController.h"

#include "IVisualizationManager.h"

#include "Tools/MoveTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                MoveTool::MoveTool(int8_t id, const std::string& name, const std::string& iconFileName)
                : Tool(id, name, iconFileName)
                {

                }

                bool MoveTool::handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt)
                {
                    // The keys have no meaning for this tool, so just forward the events to the view manager
                    evt.vizMgr->getViewController()->handleKeyEvent(evt);
                    return false;
                }

                bool MoveTool::handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    assert(evt.vizMgr != 0);
                    assert(evt.vizMgr->getViewController() != 0);

                    evt.vizMgr->getViewController()->handleMouseEvent(evt);

                    return false;
                }

            }
        }
    }
}

