#ifndef _NUMERIX_VERSION_
#define _NUMERIX_VERSION_

#include <cstddef>

/** \addtogroup core
 *  @{
 */

namespace numerix {

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

} // namespace numerix

/** @}*/

#endif // _NUMERIX_VERSION_
