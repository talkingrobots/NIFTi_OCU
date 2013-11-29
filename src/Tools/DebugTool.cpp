// Benoit 27 October 2010

#include "IVisualizationManager.h"

#include "Tools/DebugTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                DebugTool::DebugTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl)
                : StandardOCUTool(id, name, iconFileName, publisherViewControl)
                {
                    setCursorCamera("CursorQuestion", 5, 5);
                    setCursorVirtualScene("CursorQuestion", 5, 5);
                }

                bool DebugTool::onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    printf("\n%s\n", evt.vizMgr->toString().c_str());
                    return false;
                }

            }
        }
    }
}

