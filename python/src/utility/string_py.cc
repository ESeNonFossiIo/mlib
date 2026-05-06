#include <mlib/core/export.h>
#include <mlib/utility/string.h>

#include "_python/status.h"

#include <cstring>
#include <cstdint>

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_LowerCase(
    const char*   in,       ///< [in]  null-terminated input string
    char*         out,      ///< [out] output buffer
    const MLIBInt buf_size  ///< [in]  size of output buffer including null terminator
)
{
    const std::string result = mlib::lower_case(std::string(in));
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_UpperCase(
    const char*   in,       ///< [in]  null-terminated input string
    char*         out,      ///< [out] output buffer
    const MLIBInt buf_size  ///< [in]  size of output buffer including null terminator
)
{
    const std::string result = mlib::upper_case(std::string(in));
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_StrToDouble(
    const char* in,     ///< [in]  null-terminated string
    double*     result  ///< [out]
)
{
    *result = mlib::from_str_to_double(std::string(in));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_DoubleToStr(
    const double  in,       ///< [in]  value to convert
    char*         out,      ///< [out] output buffer
    const MLIBInt buf_size  ///< [in]  size of output buffer
)
{
    const std::string result = mlib::from_double_to_str(in);
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_StrToInt(
    const char* in,     ///< [in]  null-terminated string
    int64_t*    result  ///< [out]
)
{
    *result = static_cast<int64_t>(mlib::from_str_to_int(std::string(in)));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
// result is set to 1 for true, 0 for false.
MLIB_EXPORT MLIBStatus MLIB_StrToBool(
    const char* in,     ///< [in]  null-terminated string ("true"/"false"/…)
    MLIBInt*    result  ///< [out] 1 = true, 0 = false
)
{
    *result = mlib::from_str_to_bool(std::string(in)) ? 1u : 0u;
    return MLIBStatus::Success;
}
