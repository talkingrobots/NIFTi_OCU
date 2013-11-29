// Benoit 2011-05-20

#ifndef EU_NIFTI_OCU_TOOLS_CLICK_TOOL_H
#define EU_NIFTI_OCU_TOOLS_CLICK_TOOL_H

#include "Tools/StandardOCUTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {
                class SelectionManager;
            }

            namespace tools
            {

                /**
                 * This tool is the default one and allows to select EOI
                 */
                class ClickTool : public StandardOCUTool
                {
                public:
                    ClickTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, eu::nifti::ocu::selection::SelectionManager* selMgr);

                    void deactivate();

                protected:

                    bool onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    eu::nifti::ocu::selection::SelectionManager* selMgr;
                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_TOOLS_CLICK_TOOL_H

