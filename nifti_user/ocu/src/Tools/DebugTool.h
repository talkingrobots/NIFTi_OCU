// Benoit 27 October 2010

#ifndef EU_NIFTI_OCU_TOOLS_DEBUG_TOOL_H
#define EU_NIFTI_OCU_TOOLS_DEBUG_TOOL_H

#include "Tools/StandardOCUTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                /**
                 * Displays information about the visualization panels
                 */
                class DebugTool : public StandardOCUTool
                {
                public:
                    DebugTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl);

                protected:

                    bool onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_TOOLS_DEBUG_TOOL_H

