#include "numerix/utility/types.h"

#include <string>

namespace numerix {
template <typename TYPE>
TYPE zero()
{
    return static_cast<TYPE>(0);
}

template <>
std::string zero()
{
    return "";
} // GCOVR_EXCL_LINE — gcov phantom-line on closing brace of string-returning function

template <>
bool zero()
{
    return true;
}

template double zero<double>();
template float zero<float>();
template int zero<int>();
template std::size_t zero<std::size_t>();
} // namespace numerix
