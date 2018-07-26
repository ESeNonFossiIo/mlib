#include "mlib/utility/types.h"

#include <string>

namespace mlib
{
  template<typename TYPE>
  TYPE
  zero()
  {
    return 0.0;
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
    return 0;
  }

  template double zero<double>();
  template float zero<float>();
  template int zero<int>();
  template unsigned int zero<unsigned int>();
}
