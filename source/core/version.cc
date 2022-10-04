#include "mlib/core/version.h"
#include "source/_macro/string.h"


/** \addtogroup core
 *  @{
 */

namespace mlib
{

/// ----------------------------------------------------------------------------
const char* version(){return STRVALUE(MLIB_VERSION_VERSION);}

/// ----------------------------------------------------------------------------
const std::size_t version_major(){return MLIB_VERSION_VERSION_MAJOR;}

/// ----------------------------------------------------------------------------
const std::size_t version_minor(){return MLIB_VERSION_VERSION_MINOR;}

/// ----------------------------------------------------------------------------
const std::size_t version_patch(){return MLIB_VERSION_VERSION_PATCH;}

};


/** @}*/
