// Benoit 2011-05-20

#include "Selection/SelectionManager.h"

#include "Tools/ClickTool.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                ClickTool::ClickTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, eu::nifti::ocu::selection::SelectionManager* selMgr)
                : StandardOCUTool(id, name, iconFileName, publisherViewControl)
                , selMgr(selMgr)
                {
                    assert(selMgr != NULL);
                }

                void ClickTool::deactivate()
                {
                    selMgr->deselect(); // Todo May: Ensure that this is  called when we close the application, and that the car selection is deselected properly
                }

                bool ClickTool::onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    // Selects in an area of 10 X 10 pixel (otherwise it's hard with the 3D icons)
                    selMgr->select(evt.vizMgr, evt.evt.GetX(), evt.evt.GetY(), 5);
                    return false;
                }

            } // end class
        }
    }
}

