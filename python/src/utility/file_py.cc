#include <numerix/core/export.h>
#include <numerix/utility/file.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
// result is set to 1 if the file exists, 0 otherwise.
NUMERIX_EXPORT NUMERIXStatus NUMERIX_FileExists(const char* path, ///< [in]  null-terminated file path
                                       NUMERIXInt* result   ///< [out] 1 = exists, 0 = not found
)
{
    *result = numerix::file_exists(std::string(path)) ? 1u : 0u;
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_GetNumberOfLines(const char* path, ///< [in]  null-terminated file path
                                             NUMERIXInt* result   ///< [out] number of lines
)
{
    *result = static_cast<NUMERIXInt>(numerix::get_number_of_lines(std::string(path)));
    return NUMERIXStatus::Success;
}
