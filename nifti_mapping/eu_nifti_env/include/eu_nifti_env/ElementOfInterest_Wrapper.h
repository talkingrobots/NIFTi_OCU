// Benoit 2011-04-07

#ifndef EU_NIFTI_ENV_EOI_WRAPPER_H
#define EU_NIFTI_ENV_EOI_WRAPPER_H

#include <eu_nifti_env/ElementOfInterest.h>

namespace eu_nifti_env
{

    class ElementOfInterest_Wrapper
    {
    public:

        ElementOfInterest_Wrapper(ElementOfInterest* eoi);

        inline ElementOfInterest* getElementOfInterest() const
        {
            return eoi;
        }

    protected:

        ElementOfInterest* eoi;

    };



}

#endif // EU_NIFTI_ENV_EOI_WRAPPER_H