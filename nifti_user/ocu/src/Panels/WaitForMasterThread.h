// Benoit 2011-10-31

#ifndef EU_NIFTI_OCU_GUI_WAIT_FOR_MASTER_THREAD_H_
#define EU_NIFTI_OCU_GUI_WAIT_FOR_MASTER_THREAD_H_

#include <wx/thread.h>

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                class WaitForMasterDialog;
                
                class WaitForMasterThread : public wxThread
                {
                public:

                    WaitForMasterThread(WaitForMasterDialog* dialog);
                    
                    bool checkROSOnce();
                    
                    void stopChecking();
                    
                protected:

                    void* Entry();

                    int checkNumber;
                    
                    bool keepChecking;
                    
                    WaitForMasterDialog* dialog;

                };
                
                
            }
        }
    }
}

#endif // EU_NIFTI_OCU_GUI_WAIT_FOR_MASTER_THREAD_H_