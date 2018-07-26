#ifndef __MYLIB_OUTPUT_H__
#define __MYLIB_OUTPUT_H__

#include <iostream>
#include <vector>

/** \addtogroup utility
 *  @{
 */

namespace _mlib
{

  /**
   * Implement the << operator for std::vectors
   */
  template<typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
  {
    os << "[";
    if(v.size() > 0)
      {
        for(size_t i = 0; i < v.size() - 1; ++i)
          {
            os << v[i];
            os << ", ";
          }
        os << v[v.size() - 1];
      }
    os << "]";
    return os;
  }

}

/** @}*/
#endif //__OUTPUT__
