#include "mlib/utility/io.h"

#include <fstream>      // std::ifstream

namespace mlib
{

  void
  clean_screen()
  {
#ifdef _WIN32
    std::system("CLS");
#else //_WIN32

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    std::system("clear");
    ; /* this specific unused-result warning gets ignored during compilation */
#pragma GCC diagnostic pop

#endif //_WIN32
  }

}
