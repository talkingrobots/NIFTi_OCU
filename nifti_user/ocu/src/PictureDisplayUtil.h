// Benoit 2012-07-11

#ifndef EU_NIFTI_OCU_PICTURE_DISPLAY_UTIL_H
#define EU_NIFTI_OCU_PICTURE_DISPLAY_UTIL_H

#include <wx/bitmap.h>
#include <wx/image.h>

namespace eu
{
    namespace nifti
    {
        namespace gui
        {

            class PictureDisplayUtil
            {
            public:
                static wxBitmap shrink(const wxBitmap& img, int max_width, int max_height);
                static void shrink(wxImage* img, int max_width, int max_height);

                static wxBitmap rescale(const wxBitmap& img, int width, int height);
                static void rescale(wxImage* img, int width, int height);
                
            private:
                static void findNewShrinkedSize(int original_width, int original_height, int max_width, int max_height, int& new_width, int& new_height);
            };
        }

    }
}

#endif // EU_NIFTI_OCU_PICTURE_DISPLAY_UTIL_H
