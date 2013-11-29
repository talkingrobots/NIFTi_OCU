//Benoit 2013-01-28

#ifndef EU_NIFTI_OCU_GUI_I_PULL_HANDLE_EVENT_HANDLER_H
#define EU_NIFTI_OCU_GUI_I_PULL_HANDLE_EVENT_HANDLER_H

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                class PullHandle;
                
                /**
                 * Object that can handle events generated from a pull handle
                 */
                class IPullHandleEventHandler
                {
                public:
                    virtual void onPullHandleClosed(PullHandle* handle) = 0;
                    virtual void onPullHandleOpened(PullHandle* handle) = 0;
                    virtual void onPullHandleClicked(PullHandle* handle) = 0;
                };

            }
        }
    }
}

#endif //EU_NIFTI_OCU_GUI_I_PULL_HANDLE_EVENT_HANDLER_H
