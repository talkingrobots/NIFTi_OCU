// Benoit 2012-05-21

#ifndef EU_NIFTI_OCU_TOOLS_GOTO_TOOL_H
#define EU_NIFTI_OCU_TOOLS_GOTO_TOOL_H

#include <ros/publisher.h>

#include "Tools/StandardOCUTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            class VirtualSceneManager;

            namespace tools
            {

                class GoToTool : public StandardOCUTool
                {
                public:
                    GoToTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, eu::nifti::ocu::VirtualSceneManager* virtualSceneMgr);

                protected:
                    bool onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    void publishGoal(double x, double y, double theta);

                    eu::nifti::ocu::VirtualSceneManager* virtualSceneMgr;

                    ros::Publisher publisherNavGoal;
                };

            }
        }
    }
}

#endif //EU_NIFTI_OCU_TOOLS_GOTO_TOOL_H


