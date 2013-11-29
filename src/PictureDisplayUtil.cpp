// Benoit 2012-07-11

#include "PictureDisplayUtil.h"

namespace eu
{
    namespace nifti
    {
        namespace gui
        {

            /*
             * A function to resize the width/height of a wxBitmap proportionally
             */
            wxBitmap PictureDisplayUtil::shrink(const wxBitmap& img, int max_width, int max_height)
            {
                int new_width, new_height;
                findNewShrinkedSize(img.GetWidth(), img.GetHeight(), max_width, max_height, new_width, new_height);
                return rescale(img, new_width, new_height);
            }

            /*
             * A function to resize the width/height of a wxImage proportionally (based on the above)
             */
            void PictureDisplayUtil::shrink(wxImage* img, int max_width, int max_height)
            {
                int new_width, new_height;
                findNewShrinkedSize(img->GetWidth(), img->GetHeight(), max_width, max_height, new_width, new_height);
                rescale(img, new_width, new_height);
            }

            /*
             * A function to resize the width/height of a wxBitmap
             */
            wxBitmap PictureDisplayUtil::rescale(const wxBitmap& img, int width, int height)
            {
                return img.Rescale(0, 0, img.GetWidth(), img.GetHeight(), width, height);
            }

            /*
             * A function to resize the width/height of a wxImage
             */
            void PictureDisplayUtil::rescale(wxImage* img, int width, int height)
            {
                img->Rescale(width, height);
            }

            void PictureDisplayUtil::findNewShrinkedSize(int original_width, int original_height, int max_width, int max_height, int& new_width, int& new_height)
            {
                // if the width is bigger than the height, we should determine a scale for the height
                if (original_width >= original_height && original_width > max_width)
                {
                    float height_scale = (float) max_width / (float) original_width; // determine the scale

                    new_width = max_width;
                    new_height = (int) (original_height * height_scale);
                }
                else if (original_height >= original_width && original_height > max_height)
                {
                    float width_scale = (float) max_height / (float) original_height;

                    new_width = (int) (original_width * width_scale);
                    new_height = max_height;
                }
                else
                {
                    new_width = original_width;
                    new_height = original_height;
                }
            }
        }
    }
}
