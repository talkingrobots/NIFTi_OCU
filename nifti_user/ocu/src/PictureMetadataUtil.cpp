// Benoit 2012-07-11

#include <vector>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>

#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>

#include "NIFTiROSUtil.h"

#include "PictureMetadataUtil.h"

namespace fs = boost::filesystem;

namespace eu
{
    namespace nifti
    {
        namespace picviewer
        {

            // Static initializations
            bool PictureMetadataUtil::publisherIsInitialized = false;
            ros::Publisher PictureMetadataUtil::publisher;

            void PictureMetadataUtil::loadPictureMetadata(const boost::filesystem::path& path, std::vector<std::string>& metadata)
            {
                fs::ifstream fileStream(path);
                while (fileStream)
                {
                    std::string buffer;
                    std::getline(fileStream, buffer);
                    if (fileStream)
                    {
                        //std::cout << "Read in: " << buffer << "\n";
                        metadata.push_back(buffer);
                    }
                }
            }

            void PictureMetadataUtil::savePictureMetadata(const boost::filesystem::path& path, const std::vector<std::string>& metadata)
            {
                printf("Here will save the data\n");
            }

            void PictureMetadataUtil::publishPicturePose(const std::vector<std::string>& metadata)
            {

                // 1) Check for completeness

                if (metadata.size() < 7)
                {
                    std::cerr << "Error in the metadata: it does not contain the required 7 fields" << std::endl;
                    resetPictureMetadata();
                    return;
                }


                // 2) Convert the data

                geometry_msgs::PoseStamped message;

                message.header.frame_id = "/map";

                message.pose.position.x = atof(metadata.at(0).c_str());
                message.pose.position.y = atof(metadata.at(1).c_str());
                message.pose.position.z = atof(metadata.at(2).c_str());
                message.pose.orientation.w = atof(metadata.at(3).c_str());
                message.pose.orientation.x = atof(metadata.at(4).c_str());
                message.pose.orientation.y = atof(metadata.at(5).c_str());
                message.pose.orientation.z = atof(metadata.at(6).c_str());


                // 3) Publish the message

                if (publisherIsInitialized == false)
                {
                    publisher = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<geometry_msgs::PoseStamped > ("/PicViewer/Pose", 1);
                    publisherIsInitialized = true;
                }

                publisher.publish(message);
            }

            void PictureMetadataUtil::resetPictureMetadata()
            {
                geometry_msgs::PoseStamped message;

                message.header.frame_id = "/map";
                message.pose.position.z = 9999; // Tries to hide the arrow

                // DEBUGGING BLOCK
                //                    message.pose.position.x = 0;
                //                    message.pose.position.y = 0;
                //                    message.pose.position.z = 9999;
                //                    message.pose.orientation.w = 0.707106781;
                //                    message.pose.orientation.x = 0;
                //                    message.pose.orientation.y = -0.707106781;
                //                    message.pose.orientation.z = 0;


                if (publisherIsInitialized == false)
                {
                    publisher = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<geometry_msgs::PoseStamped > ("/PicViewer/Pose", 1);
                    publisherIsInitialized = true;
                }

                publisher.publish(message);
            }

            // So far, this is only a debugging method

            void PictureMetadataUtil::publishPose(double w, double x, double y, double z)
            {
                geometry_msgs::PoseStamped message;

                message.header.frame_id = "/map";

                // position is 0,0,0

                message.pose.orientation.w = w;
                message.pose.orientation.x = x;
                message.pose.orientation.y = y;
                message.pose.orientation.z = z;


                if (publisherIsInitialized == false)
                {
                    publisher = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<geometry_msgs::PoseStamped > ("/PicViewer/Pose", 1);
                    publisherIsInitialized = true;
                }

                publisher.publish(message);
            }

            // http://thisthread.blogspot.de/2011/07/listing-files-in-directory.html

            void PictureMetadataUtil::listFilesInDirectory(boost::filesystem::path directory, std::set<boost::filesystem::path>& files)
            {
                files.clear();

                boost::filesystem::directory_iterator beg(directory);
                boost::filesystem::directory_iterator end;
                std::copy(beg, end, std::inserter(files, files.begin())); // 1.
            }

        }
    }
}
