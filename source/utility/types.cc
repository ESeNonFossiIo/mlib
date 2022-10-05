#include "mlib/utility/types.h"

#include <string>

namespace mlib
{
  template<typename TYPE>
  TYPE
  zero()
  {
    return static_cast<TYPE>(0);
  }

  template<>
  std::string
  zero()
  {
    return "";
  }

  template<>
  bool
  zero()
  {
    return true;
  }

  template double zero<double>();
  template float zero<float>();
  template int zero<int>();
  template std::size_t zero<std::size_t>();
}
