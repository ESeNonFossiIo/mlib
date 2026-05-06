#include <mlib/core/export.h>
#include <mlib/utility/file.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
// result is set to 1 if the file exists, 0 otherwise.
MLIB_EXPORT MLIBStatus MLIB_FileExists(
    const char* path,   ///< [in]  null-terminated file path
    MLIBInt*    result  ///< [out] 1 = exists, 0 = not found
)
{
    *result = mlib::file_exists(std::string(path)) ? 1u : 0u;
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_GetNumberOfLines(
    const char* path,   ///< [in]  null-terminated file path
    MLIBInt*    result  ///< [out] number of lines
)
{
    *result = static_cast<MLIBInt>(mlib::get_number_of_lines(std::string(path)));
    return MLIBStatus::Success;
}
