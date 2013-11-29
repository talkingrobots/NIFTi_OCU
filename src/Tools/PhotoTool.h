// Benoit 2011-11-22

#ifndef EU_NIFTI_OCU_TOOLS_PHOTO_TOOL_H
#define EU_NIFTI_OCU_TOOLS_PHOTO_TOOL_H

#include <image_transport/image_transport.h>

#include "Tools/StandardOCUTool.h"

namespace Ogre
{
    class Image;
    class Quaternion;
    class SceneManager;
    class Vector3;
}

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {

                /**
                 * This tool takes a photo of a camera when the user clicks on
                 * a view. For now, works only with camera feeds and not virtual
                 * scenes.
                 */
                class PhotoTool : public StandardOCUTool
                {
                public:
                    PhotoTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, wxWindow* parentWindow);

                    bool onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                protected:

                    /**
                     * Returns true if a picture was taken
                     * @param evt
                     * @return 
                     */
                    bool takePhoto(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    void publishImageToROS(const int sequenceNumber, const std::string& referenceFrame, const ros::Time& timeStamp, const Ogre::Image& ogreImage, const std::string& imageEncoding);

                    void createMetadata(std::string& metadata, const ros::Time& timeStamp, Ogre::Vector3 position, Ogre::Quaternion orientation, double fieldOfViewHorizontal, double fieldOfViewVertical, const std::string& annotation);

                    void publishMetadataToROS(const int sequenceNumber, const std::string& metadata, const ros::Time& timeStamp);

                    image_transport::ImageTransport imageTransport;
                    image_transport::Publisher publisherImage;

                    ros::Publisher publisherMetadata;

                    wxWindow* parentWindow; // Used for the annotation dialog

                    static const std::string ROS_TOPIC_IMAGE;
                    static const std::string ROS_TOPIC_METADATA;

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_TOOLS_PHOTO_TOOL_H

