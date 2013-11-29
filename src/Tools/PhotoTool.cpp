// Benoit 2011-11-22

#include <sstream>

#include <boost/filesystem/fstream.hpp>

#include <wx/msgdlg.h> 
#include <wx/textdlg.h>

#include "IVisualizationManager.h"
#include "NIFTiROSOgreUtil.h"
#include "NIFTiROSUtil.h"
#include "NIFTiConstants.h"

#include "ViewControllers/IViewController.h"

#include "Tools/PhotoTool.h"

using namespace std;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace tools
            {
                const std::string PhotoTool::ROS_TOPIC_IMAGE = "ocu/screenshot";
                const std::string PhotoTool::ROS_TOPIC_METADATA = "ocu/screenshot/metadata";

                PhotoTool::PhotoTool(int8_t id, const std::string& name, const std::string& iconFileName, const ros::Publisher* publisherViewControl, wxWindow* parentWindow)
                : StandardOCUTool(id, name, iconFileName, publisherViewControl)
                , imageTransport(*eu::nifti::ocu::NIFTiROSUtil::getNodeHandle())
                , parentWindow(parentWindow)
                {
                    setCursorCamera("CursorPhoto", CURSOR_SIZE / 2 - 1, CURSOR_SIZE / 2 - 1);

                    publisherImage = imageTransport.advertise(ROS_TOPIC_IMAGE, 10, false);
                    publisherMetadata = NIFTiROSUtil::getNodeHandle()->advertise<std_msgs::Header > (ROS_TOPIC_METADATA, 10, false);
                }

                bool PhotoTool::onShortClick(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    return takePhoto(evt);
                }

                bool PhotoTool::takePhoto(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)
                {
                    //ROS_WARN("IN bool PhotoTool::takePhoto(eu::nifti::ocu::gui::MultiVizMouseEvent& evt)");

                    if (!NIFTiViewsUtil::viewTypeIsCamera(evt.vizMgr->getViewType()))
                    {
                        // Do nothing
                        // Eventually, find a way to capture the Ogre contents
                        return false;
                    }

                    wxBell(); // Beeps to give a confirmation to the user

                    
                    Ogre::Image ogreImage;
                    try
                    {
                        // Gets the current displayed image (**without overlays**)
                        ogreImage = evt.vizMgr->getCurrentImage();
                    }
                    catch (char const* ex)
                    {
                        std::cerr << ex << std::endl;
                        std::cerr << "Could not take a snapshot of this camera because the resolution is too high. You can change it in omnicamera -- load_pano.launch" << std::endl;

                        wxMessageBox(wxT("Could not take a snapshot of this camera because the resolution is too high."), wxT("Snapshot Problem"));

                        return false;
                    }

                    // Ask the user for an annotation
                    wxString annotation = wxGetTextFromUser(wxT("If desired, give a description of the picture."), wxT("Description of the Picture"), wxEmptyString, parentWindow);

                    static int sequenceNumber = 0;
                    sequenceNumber++;
                    ros::Time timeStamp = ros::Time::now();
                    string metadata;

                    publishImageToROS(sequenceNumber, evt.vizMgr->getFixedFrame(), timeStamp, ogreImage, evt.vizMgr->getCurrentImageEncoding());

                    Ogre::Vector3 position;
                    Ogre::Quaternion orientation;
                    
                    try
                    {
                        position = evt.vizMgr->getViewController()->getPosition();
                        orientation = evt.vizMgr->getViewController()->getOrientation();
                    }
                    catch(const std::string& ex)
                    {
                        ROS_DEBUG_STREAM("Cannot get a position for the picture. " << ex);
                    }
                    
                    NIFTiROSOgreUtil::convertOrientationFromOgreToROS(orientation);
                    
                    createMetadata(metadata, timeStamp, position, orientation, evt.vizMgr->getFieldOfViewHorizontal(), evt.vizMgr->getFieldOfViewVertical(), std::string(annotation.mb_str()));
                    
                    publishMetadataToROS(sequenceNumber, metadata, timeStamp);
                    
                    return true;
                }
                
                void PhotoTool::publishImageToROS(const int sequenceNumber, const std::string& referenceFrame, const ros::Time& timeStamp, const Ogre::Image& ogreImage, const std::string& imageEncoding)
                {
                    sensor_msgs::Image imageMsg;

                    imageMsg.header.seq = sequenceNumber;
                    imageMsg.header.frame_id = referenceFrame;
                    imageMsg.header.stamp = timeStamp;
                    imageMsg.height = ogreImage.getHeight();
                    imageMsg.width = ogreImage.getWidth();
                    imageMsg.encoding = imageEncoding;
                    imageMsg.is_bigendian = false;
                    imageMsg.step = ogreImage.getRowSpan();
                    size_t size = imageMsg.step * imageMsg.height;
                    imageMsg.data.resize(size);

                    // Copies the data from the Ogre Image to the ROS Image
                    memcpy((char*) (&imageMsg.data[0]), ogreImage.getData(), size);

                    publisherImage.publish(imageMsg);
                }

                void PhotoTool::createMetadata(std::string& metadata, const ros::Time& timeStamp, Ogre::Vector3 position, Ogre::Quaternion orientation, double fieldOfViewHorizontal, double fieldOfViewVertical, const std::string& annotation)
                {
                    stringstream ss;
                    ss << timeStamp << std::endl;
                    ss << position.x << std::endl;
                    ss << position.y << std::endl;
                    ss << position.z << std::endl;
                    ss << orientation.w << std::endl;
                    ss << orientation.x << std::endl;
                    ss << orientation.y << std::endl;
                    ss << orientation.z << std::endl;
                    ss << orientation.getRoll(false).valueRadians() << std::endl; // OGRE camera frame looks along -Z, so they call rotation around Z "roll". This represents the orientation as viewed from top down. x-axis is zero, then goes counter-clockwise 
                    ss << fieldOfViewHorizontal << std::endl;
                    ss << fieldOfViewVertical << std::endl;

                    ss << "WRITE ONLY BELOW THIS LINE / SCRIVERE SOLO SOTTO DI QUESTA LINEA / SCHREIBEN NUR UNTER DIESER LINIE" << std::endl;
                    ss << "================================================================" << std::endl;

                    if (annotation != "")
                    {
                        ss << annotation << std::endl;
                    }

                    metadata = ss.str();
                }

                void PhotoTool::publishMetadataToROS(const int sequenceNumber, const std::string& metadata, const ros::Time& timeStamp)
                {
                    std_msgs::Header msg;

                    msg.seq = sequenceNumber;
                    msg.frame_id = metadata; // Here, I abuse the field for frame_id to put in the metadata
                    msg.stamp = timeStamp;

                    publisherMetadata.publish(msg);
                }

            } // end class
        }
    }

}

