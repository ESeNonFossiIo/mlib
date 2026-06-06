#include <numerix/core/export.h>
#include <numerix/core/version.h>

#include "_python/types.h"

/// ----------------------------------------------------------------------------
/// Get the current version
NUMERIX_EXPORT const char* NUMERIX_version()
{
    return numerix::version();
}

/// ----------------------------------------------------------------------------
/// Get the current major version
NUMERIX_EXPORT NUMERIXInt NUMERIX_version_major()
{
    return static_cast<NUMERIXInt>(numerix::version_major());
}

/// ----------------------------------------------------------------------------
/// Get the current minor version
NUMERIX_EXPORT NUMERIXInt NUMERIX_version_minor()
{
    return static_cast<NUMERIXInt>(numerix::version_minor());
}

/// ----------------------------------------------------------------------------
/// Get the current patch version
NUMERIX_EXPORT NUMERIXInt NUMERIX_version_patch()
{
    return static_cast<NUMERIXInt>(numerix::version_patch());
}
