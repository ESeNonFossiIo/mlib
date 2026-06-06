#include <numerix/core/string.h>
#include <numerix/core/version.h>

namespace numerix {

/// ----------------------------------------------------------------------------
const char* version()
{
    return STRVALUE(NUMERIX_VERSION_VERSION);
}

/// ----------------------------------------------------------------------------
std::size_t version_major()
{
    return NUMERIX_VERSION_VERSION_MAJOR;
}

/// ----------------------------------------------------------------------------
std::size_t version_minor()
{
    return NUMERIX_VERSION_VERSION_MINOR;
}

/// ----------------------------------------------------------------------------
std::size_t version_patch()
{
    return NUMERIX_VERSION_VERSION_PATCH;
}

} // namespace numerix
