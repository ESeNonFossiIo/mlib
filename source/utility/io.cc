#include "mlib/utility/io.h"

#include <fstream>      // std::ifstream

namespace _mlib
{

  void
  clean_screen()
  {
#ifdef _WIN32
    std::system("CLS");
#else //_WIN32
    std::system("clear");
#endif //_WIN32
  }

}
