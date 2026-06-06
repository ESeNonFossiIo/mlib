#include <numerix/core/export.h>
#include <numerix/utility/string.h>

#include "_python/status.h"

#include <cstdint>
#include <cstring>

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_LowerCase(
    const char* in,        ///< [in]  null-terminated input string
    char* out,             ///< [out] output buffer
    const NUMERIXInt buf_size ///< [in]  size of output buffer including null terminator
)
{
    const std::string result = numerix::lower_case(std::string(in));
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_UpperCase(
    const char* in,        ///< [in]  null-terminated input string
    char* out,             ///< [out] output buffer
    const NUMERIXInt buf_size ///< [in]  size of output buffer including null terminator
)
{
    const std::string result = numerix::upper_case(std::string(in));
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_StrToDouble(const char* in, ///< [in]  null-terminated string
                                        double* result  ///< [out]
)
{
    *result = numerix::from_str_to_double(std::string(in));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_DoubleToStr(const double in,       ///< [in]  value to convert
                                        char* out,             ///< [out] output buffer
                                        const NUMERIXInt buf_size ///< [in]  size of output buffer
)
{
    const std::string result = numerix::from_double_to_str(in);
    std::strncpy(out, result.c_str(), static_cast<std::size_t>(buf_size) - 1);
    out[buf_size - 1] = '\0';
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_StrToInt(const char* in, ///< [in]  null-terminated string
                                     int64_t* result ///< [out]
)
{
    *result = static_cast<int64_t>(numerix::from_str_to_int(std::string(in)));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
// result is set to 1 for true, 0 for false.
NUMERIX_EXPORT NUMERIXStatus NUMERIX_StrToBool(
    const char* in, ///< [in]  null-terminated string ("true"/"false"/…)
    NUMERIXInt* result ///< [out] 1 = true, 0 = false
)
{
    *result = numerix::from_str_to_bool(std::string(in)) ? 1u : 0u;
    return NUMERIXStatus::Success;
}
