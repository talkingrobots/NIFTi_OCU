// Benoit 2011-06-30

#include <eu_nifti_env/CarObjectOfInterest.h>
#include <eu_nifti_env/SignObjectOfInterest.h>
#include <eu_nifti_env/VictimObjectOfInterest.h>

#include <eu_nifti_env_msg_ros/Util.h>

namespace eu
{
    namespace nifti
    {
        namespace env
        {
            //typedef int16_t UUID;

            namespace msg
            {
                namespace ros
                {

                    // This constructor is private, because I want this to be an abstract class

                    Util::Util()
                    {
                    }

                    eu::nifti::env::UUID Util::getUUID(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                    {
                        switch (msg->type)
                        {
                            case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                                return msg->car.object.element.uuid;
                            case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                                return msg->sign.object.element.uuid;
                            case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                                return msg->victim.object.element.uuid;
                            default:
                                std::cerr << "Object of Interest is of unknown type: " << msg->type << std::endl;
                                throw "Object of Interest is of unknown type";
                        }
                    }

                    const eu_nifti_env::ObjectOfInterest* Util::getOOI(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr& msg)
                    {
                        switch (msg->type)
                        {
                            case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                                return &(msg->car.object);
                            case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                                return &(msg->sign.object);
                            case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                                return &(msg->victim.object);
                            default:
                                std::cerr << "Object of Interest is of unknown type: " << msg->type << std::endl;
                                throw "Object of Interest is of unknown type";
                        }
                    }

                }
            }
        }
    }
}
