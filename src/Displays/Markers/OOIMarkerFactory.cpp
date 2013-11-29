// Benoit 2011-06-08

#include <eu_nifti_env/CarObjectOfInterest.h>
#include <eu_nifti_env/SignObjectOfInterest.h>
#include <eu_nifti_env/VictimObjectOfInterest.h>

#include "Displays/Markers/CarObjectOfInterestMarker.h"
#include "Displays/Markers/SignObjectOfInterestMarker.h"
#include "Displays/Markers/VictimObjectOfInterestMarker.h"

#include "Displays/Markers/OOIMarkerFactory.h"

namespace eu
{
    namespace nifti
    {
        namespace env
        {
            typedef int16_t UUID;
        }

        namespace ocu
        {
            namespace display
            {
                Ogre::SceneManager* OOIMarkerFactory::sceneMgr = NULL;
                rviz::FrameTransformer* OOIMarkerFactory::frameTransformer = NULL;
                Ogre::SceneNode* OOIMarkerFactory::sceneNode = NULL;

                // This constructor is private, because I want this to be an abstract class
                OOIMarkerFactory::OOIMarkerFactory()
                {
                }

                void OOIMarkerFactory::initialize(Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameTransformer, Ogre::SceneNode* sceneNode)
                {
                    OOIMarkerFactory::sceneMgr = sceneMgr;
                    OOIMarkerFactory::frameTransformer = frameTransformer;
                    OOIMarkerFactory::sceneNode = sceneNode;
                }
                
                ObjectOfInterestMarkerPtr OOIMarkerFactory::getOOIMarker(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                {
                    ObjectOfInterestMarker* marker = NULL;

                    switch (msg->type)
                    {
                        case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                            marker = new CarObjectOfInterestMarker(msg, sceneMgr, frameTransformer, sceneNode);
                            break;
                        case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                            marker = new SignObjectOfInterestMarker(msg, sceneMgr, frameTransformer, sceneNode);
                            break;
                        case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                            marker = new VictimObjectOfInterestMarker(msg, sceneMgr, frameTransformer, sceneNode);
                            break;
                        default:
                            std::cerr << "Object of Interest is of unknown type: " << msg->type << std::endl;
                            throw "Object of Interest is of unknown type";
                    }
                    
                    return ObjectOfInterestMarkerPtr(marker);
                }


            }
        }
    }
}
