// Benoit 2010-09-28

#ifndef NIFTI_ANNOTATED_STATUS_LEVEL_H
#define NIFTI_ANNOTATED_STATUS_LEVEL_H

#include <string>

namespace eu
{

    namespace nifti
    {

        namespace ocu
        {

            /**
             * Three-level status gradation
             */
            enum StatusLevel
            {
                STATUS_LEVEL_OK, STATUS_LEVEL_WARNING, STATUS_LEVEL_ERROR
            };

            /**
             * Indicates a status with a textual explanation. The level is mostly to
             * help with the program logic, and the message is mostly to help with
             * debugging.
             */
            struct AnnotatedStatusLevel
            {

                AnnotatedStatusLevel(StatusLevel level, std::string msg)
                {
                    this->level = level;
                    this->msg = msg;
                }

                StatusLevel level;
                std::string msg;
            };

        }
    }
}

#endif // NIFTI_ANNOTATED_STATUS_LEVEL_H