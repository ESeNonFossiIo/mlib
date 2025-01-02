#include <mlib/core/version.h>
#include <mlib/core/export.h>

#include "_python/types.h"

/// ----------------------------------------------------------------------------
/// Get the current version
MLIB_EXPORT const char* MLIB_version()
{
  return mlib::version();
}

/// ----------------------------------------------------------------------------
/// Get the current major version
MLIB_EXPORT MLIBInt MLIB_version_major()
{
  return static_cast<MLIBInt>(mlib::version_major());
}

/// ----------------------------------------------------------------------------
/// Get the current minor version
MLIB_EXPORT MLIBInt MLIB_version_minor()
{
  return static_cast<MLIBInt>(mlib::version_minor());
}

/// ----------------------------------------------------------------------------
/// Get the current patch version
MLIB_EXPORT MLIBInt MLIB_version_patch()
{
  return static_cast<MLIBInt>(mlib::version_patch());
}
