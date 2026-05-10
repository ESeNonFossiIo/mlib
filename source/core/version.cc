#include <mlib/core/string.h>
#include <mlib/core/version.h>

namespace mlib {

/// ----------------------------------------------------------------------------
const char* version()
{
    return STRVALUE(MLIB_VERSION_VERSION);
}

/// ----------------------------------------------------------------------------
std::size_t version_major()
{
    return MLIB_VERSION_VERSION_MAJOR;
}

/// ----------------------------------------------------------------------------
std::size_t version_minor()
{
    return MLIB_VERSION_VERSION_MINOR;
}

/// ----------------------------------------------------------------------------
std::size_t version_patch()
{
    return MLIB_VERSION_VERSION_PATCH;
}

} // namespace mlib
