// Benoit 2011-04-07

#ifndef EU_NIFTI_ENV_LOI_WRAPPER_H
#define EU_NIFTI_ENV_LOI_WRAPPER_H

#include "eu_nifti_env/ElementOfInterest_Wrapper.h"
#include "eu_nifti_env/LocationOfInterest.h"

namespace eu_nifti_env
{

    class LocationOfInterest_Wrapper : public ElementOfInterest_Wrapper
    {
    public:

        LocationOfInterest_Wrapper(LocationOfInterest* loi);

        inline LocationOfInterest* getLocationOfInterest() const
        {
            return loi;
        }

    protected:

        LocationOfInterest* loi;

    };

}

#endif // EU_NIFTI_ENV_LOI_WRAPPER_H