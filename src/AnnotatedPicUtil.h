// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-02-19

#ifndef EU_NIFTI_OCU_ANNOTATED_PIC_UTIL_H
#define EU_NIFTI_OCU_ANNOTATED_PIC_UTIL_H

#include "EXIFReader_msgs/AnnotatedPicture.h"

namespace Ogre
{
    class Image;
}

class wxImage;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            /**
             * Utilities for annotated pictures (contain EXIF data with GPS info)
             * @param folder
             */
            class AnnotatedPicUtil
            {
            public:

                static wxImage* createWxImage(const EXIFReader_msgs::AnnotatedPicture* picture);
                static Ogre::Image* createOgreImage(const EXIFReader_msgs::AnnotatedPicture* picture);
            };


        }
    }
}

#endif // EU_NIFTI_OCU_ANNOTATED_PIC_UTIL_H
