// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-02-19

#include <wx/image.h>
#include <wx/mstream.h>

#include <OGRE/OgreDataStream.h>
#include <OGRE/OgreImage.h>

#include"AnnotatedPicUtil.h"

using namespace std;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {

            wxImage* AnnotatedPicUtil::createWxImage(const EXIFReader_msgs::AnnotatedPicture* picture)
            {
                wxMemoryInputStream dataStream(picture->completeFile.data(), picture->completeFile.size());
                wxImage* img = new wxImage();

                if (!img->LoadFile(dataStream))
                {
                    cerr << "There was a problem loading picture: " << picture->filename << endl;
                    delete img;
                    return NULL;
                }

                return img;
            }

            Ogre::Image* AnnotatedPicUtil::createOgreImage(const EXIFReader_msgs::AnnotatedPicture* picture)
            {
                assert(picture != NULL);
                assert(picture->completeFile.size() != 0);
                
                //ROS_INFO_STREAM("IN Ogre::Image* AnnotatedPicUtil::createOgreImage(const EXIFReader_msgs::AnnotatedPicture* picture): " << picture->filename << " of size " << picture->completeFile.size());
                
                Ogre::DataStreamPtr dataStream(new Ogre::MemoryDataStream((void*) picture->completeFile.data(), picture->completeFile.size(), false, true));
                
                Ogre::Image* img = new Ogre::Image();
                img->load(dataStream);
                
                //ROS_INFO_STREAM("OUT Ogre::Image* AnnotatedPicUtil::createOgreImage(const EXIFReader_msgs::AnnotatedPicture* picture): " << picture->filename << " of size " << picture->completeFile.size());
                
                return img;
            }
        }
    }
}




//
//http://poor3d.googlecode.com/svn-history/r34/trunk/Poor3D/Engine/MemoryManager/nedmalloc.h
//
//
///* Gets the usable size of an allocated block. Note this will always be bigger than what was
//asked for due to rounding etc. Optionally returns 1 in isforeign if the block came from the
//system allocator - note that there is a small (>0.01%) but real chance of segfault on non-Windows
//systems when passing non-nedmalloc blocks if you don't use USE_MAGIC_HEADERS.
//*/
//NEDMALLOCEXTSPEC NEDMALLOCNOALIASATTR size_t nedblksize(int *RESTRICT isforeign, void *RESTRICT mem) THROWSPEC;
//
//
//
//
//ISO C (and thus also C++) allows you to catch and handle the SIGSEGV
//signal (SEGV is short for "segmentation violation"). However, the only
//sane reason to do so is to save work before aborting the program; the
//behavior of a program which ignores SIGSEGV (i.e., one which continues
//running after SIGSEGV) is undefined.