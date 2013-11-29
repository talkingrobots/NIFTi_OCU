// Benoit 2011-10-31

#include <iostream>

#include <ros/master.h>

#include "Panels/WaitForMasterDialog.h"

#include "Panels/WaitForMasterThread.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace gui
            {

                WaitForMasterThread::WaitForMasterThread(WaitForMasterDialog* dialog)
                : wxThread(wxTHREAD_DETACHED)
                , checkNumber(0)
                , keepChecking(true)
                , dialog(dialog)
                {
                }

                void* WaitForMasterThread::Entry()
                {
                    //Forces a ros master check right away
                    do
                    {
                        Sleep(1000); // Checks every second
                        
                        if(!keepChecking) break; // Stops if requested to
                        
                        if (checkROSOnce()) // Stops if the connection to ROS has been established
                        {
                            break;
                        }
                        
                    } while (true);

                    return 0;
                }
                
                void WaitForMasterThread::stopChecking()
                {
                    keepChecking = false;
                }

                bool WaitForMasterThread::checkROSOnce()
                {
                    if (checkNumber > 0) // I don't know how to write this easily on the dialog, because it must be done from the main thread
                    {
                        std::cout << "Contacting ROS master at [" << ros::master::getURI() << "]. Retrial #" << checkNumber << "..." << std::endl;
                    }

                    checkNumber++;

                    if (ros::master::check())
                    {
                        dialog->rosChecked();
                        return true;
                    }

                    return false;
                }


            }
        }
    }
}