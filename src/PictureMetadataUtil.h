// Benoit 2012-07-11

#ifndef EU_NIFTI_PICVIEWER_PICTURE_METADATA_UTIL_H
#define EU_NIFTI_PICVIEWER_PICTURE_METADATA_UTIL_H

#include <boost/filesystem/path.hpp>

#include <ros/publisher.h>

namespace eu
{
    namespace nifti
    {
        namespace picviewer
        {
            class PictureMetadataUtil
            {
            public:
                static void loadPictureMetadata(const boost::filesystem::path& path, std::vector<std::string>& metadata);
                static void savePictureMetadata(const boost::filesystem::path& path, const std::vector<std::string>& metadata);
                static void publishPicturePose(const std::vector<std::string>& metadata);
                static void publishPose(double w, double x, double y, double z);
                static void resetPictureMetadata();
                static void listFilesInDirectory(boost::filesystem::path directory, std::set<boost::filesystem::path>& files);
            protected:
                static bool publisherIsInitialized;
                static ros::Publisher publisher;
                
            };
        }
    }
}

#endif // EU_NIFTI_PICVIEWER_PICTURE_METADATA_UTIL_H
