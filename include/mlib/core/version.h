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
  std::size_t version_major();

/// ----------------------------------------------------------------------------
/// Return the current minor version
  std::size_t version_minor();

/// ----------------------------------------------------------------------------
/// Return the current patch version
  std::size_t version_patch();

}


/** @}*/

#endif // _MLIB_VERSION_
