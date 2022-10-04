#ifndef _MLIB_VERSION_
#define _MLIB_VERSION_

#include <cstddef>

/** \addtogroup core
 *  @{
 */

namespace mlib
{

/// ----------------------------------------------------------------------------
/// Return teh current version (complete)
const char* version();

/// ----------------------------------------------------------------------------
/// Return the current major version
const std::size_t version_major();

/// ----------------------------------------------------------------------------
/// Return the current minor version
const std::size_t version_minor();

/// ----------------------------------------------------------------------------
/// Return the current patch version
const std::size_t version_patch();

};


/** @}*/

#endif // _MLIB_VERSION_
