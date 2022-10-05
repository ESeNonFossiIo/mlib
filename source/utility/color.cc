#include "mlib/utility/color.h"

namespace mlib
{
  GeneralColor::
  GeneralColor(const int& color_, const int& style_)
    :
    color(color_),
    style(style_)
  {}

  std::string
  GeneralColor::
  init() const
  {
// #ifndef WIN32
    return "\033["+std::to_string(style)+";"+std::to_string(color)+"m";
// #else
    // return "<Esc>["+std::to_string(style)+";"+std::to_string(color-65)+"m";
// #endif
  }

  std::string
  GeneralColor::
  end() const
  {
// #ifndef WIN32
    return "\033[0m";
// #else
    // return "<Esc>[0m";
// #endif
  }

}
