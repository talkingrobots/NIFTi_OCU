/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 */

#include <wx/app.h>
#include <wx/stattext.h>
#include <wx/timer.h>

#include "WXLogROSOut.h"
#include <ogre_tools/initialization.h>

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <signal.h>

#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <std_srvs/Empty.h>

#include "NIFTiROSUtil.h"
#include "RobotsStatusesManager.h"

#include "Panels/WaitForMasterDialog.h"
#include "Panels/WaitForMasterThread.h"

#include "Panels/OCUFrame.h"

namespace po = boost::program_options;

namespace rviz
{

    bool reloadShaders(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        Ogre::ResourceManager::ResourceMapIterator it = Ogre::HighLevelGpuProgramManager::getSingleton().getResourceIterator();
        while (it.hasMoreElements())
        {
            Ogre::ResourcePtr resource = it.getNext();
            resource->reload();
        }
        return true;
    }

    class AppOCU : public wxApp
    {
    public:
        char** local_argv_;
        OCUFrame* frame_;
        volatile bool continue_;
        boost::thread signal_handler_thread_;
        wxTimer timer_;
        ros::NodeHandlePtr nh_;
        ros::ServiceServer reload_shaders_service_;

        AppOCU()
        : timer_(this)
        {
        }

        void onTimer(wxTimerEvent&)
        {
            if (!continue_)
            {
                frame_->Close();
            }
        }

        bool OnInit()
        {
            wxLog::SetActiveTarget(new wxLogRosout());

            // create our own copy of argv, with regular char*s.
            local_argv_ = new char*[ argc ];
            for (int i = 0; i < argc; ++i)
            {
                local_argv_[ i ] = strdup(wxString(argv[ i ]).mb_str());
            }

            ros::init(argc, local_argv_, ROS_PACKAGE_NAME, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);


            ////////////////////////////////

            // Checks that the connection to the ROS server is ok
            {
                eu::nifti::ocu::gui::WaitForMasterDialog* d = new eu::nifti::ocu::gui::WaitForMasterDialog(ros::master::getURI());

                eu::nifti::ocu::gui::WaitForMasterThread* t = new eu::nifti::ocu::gui::WaitForMasterThread(d);


                if (!t->checkROSOnce()) // If I can contact the ros master directly, I don't even start the thread and window
                {
                    t->Create();
                    t->Run();

                    if (d->ShowModal() != wxID_OK)
                    {
                        t->stopChecking();
                        t->Delete();
                        delete d;

                        return false;
                    }
                }

                t->Delete();
                delete d;

            }
            ////////////////////////////////




            eu::nifti::ocu::RobotsStatusesManager::init();

            po::options_description options;
            options.add_options()
                    ("help,h", "Produce this help message")
                    ("ogre-log,l", "Enable the Ogre.log file (output in cwd)");
            po::variables_map vm;
            bool enable_ogre_log = false;
            try
            {
                po::store(po::parse_command_line(argc, local_argv_, options), vm);
                po::notify(vm);

                if (vm.count("help"))
                {
                    std::cout << "rviz command line options:\n" << options;
                    return false;
                }

                if (vm.count("ogre-log"))
                {
                    enable_ogre_log = true;
                }
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Error parsing command line: %s", e.what());
                return false;
            }

            // block kill signals on all threads, since this also disables signals in threads
            // created by this one (the main thread)
            sigset_t sig_set;
            sigemptyset(&sig_set);
            sigaddset(&sig_set, SIGKILL);
            sigaddset(&sig_set, SIGTERM);
            sigaddset(&sig_set, SIGQUIT);
            sigaddset(&sig_set, SIGINT);
            pthread_sigmask(SIG_BLOCK, &sig_set, NULL);

            // Start up our signal handler
            continue_ = true;
            signal_handler_thread_ = boost::thread(boost::bind(&AppOCU::signalHandler, this));

            nh_.reset(new ros::NodeHandle);
            ogre_tools::initializeOgre(enable_ogre_log);

            try
            {
                frame_ = new OCUFrame();
                frame_->initialize();
            }
            catch (std::exception& ex)
            {
                std::cerr << "Caught an unrecoverable exception in the initialization of the OCU frame: " << std::endl;
                std::cerr << ex.what() << std::endl;
                std::cerr << "This application will shut down." << std::endl;
                
                return false;
            }
            catch (...)
            {
                std::cerr << "Caught an unrecoverable problem in the initialization of the OCU frame. This application will shut down." << std::endl;
                return false;
            }

            SetTopWindow(frame_);
            frame_->Show();

            Connect(timer_.GetId(), wxEVT_TIMER, wxTimerEventHandler(AppOCU::onTimer), NULL, this);
            timer_.Start(100);

            ros::NodeHandle private_nh("~");
            reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);

            return true;
        }

        int OnExit()
        {
            eu::nifti::ocu::RobotsStatusesManager::unInit();

            timer_.Stop();
            continue_ = false;

            raise(SIGQUIT);

            signal_handler_thread_.join();

            for (int i = 0; i < argc; ++i)
            {
                free(local_argv_[ i ]);
            }
            delete [] local_argv_;

            ogre_tools::cleanupOgre();

            return 0;
        }

        void signalHandler()
        {
            sigset_t signal_set;
            while (continue_)
            {
                // Wait for any signals
                sigfillset(&signal_set);

                struct timespec ts = {0, 100000000};
                int sig = sigtimedwait(&signal_set, NULL, &ts);

                switch (sig)
                {
                    case SIGKILL:
                    case SIGTERM:
                    case SIGQUIT:
                    case SIGINT:
                    {
                        ros::shutdown();
                        continue_ = false;
                        return;
                    }
                        break;

                    default:
                        break;
                }
            }
        }
    };

    DECLARE_APP(AppOCU);

} // namespace rviz


IMPLEMENT_APP(rviz::AppOCU);
