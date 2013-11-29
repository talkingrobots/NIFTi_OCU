// Benoit July 2010

#ifndef EU_NIFTI_OCU_TOOLS_MOVE_TOOL_H
#define EU_NIFTI_OCU_TOOLS_MOVE_TOOL_H

#include "Tools/Tool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                class MoveTool : public Tool
                {
                public:
                    MoveTool(int8_t id, const std::string& name, const std::string& iconFileName);

                    void activate()
                    {
                    }

                    void deactivate()
                    {
                    }

                    /**
                     * Delegates the task to the view controller
                     * @param event
                     */
                    bool handleKeyEvent(eu::nifti::ocu::gui::MultiVizKeyEvent& evt);

                    /**
                     * Delegates the task to the view controller
                     * @param event
                     */
                    bool handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                };

            }
        }
    }
}

#endif

