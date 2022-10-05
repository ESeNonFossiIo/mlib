#include "mlib/utility/progress_bar.h"

namespace mlib
{

  ProgressBar::
  ProgressBar(
    std::size_t length_,
    std::string c_)
    :
    c(c_),
    length(length_)
  {}

  void
  ProgressBar::
  print_bar(std::size_t pos)
  {
    std::size_t status = (int)((double) pos*
                               (double) length/100.0);

    std::cout << "\r [";
    for(std::size_t i = 0; i<length; ++i)
      {
        if(i <= status)
          std::cout << c;
        else
          std::cout << " ";
      }
    std::cout << "] ["
              << std::setw(3)
              << pos << "%]\r" << std::flush;
  }

  void
  ProgressBar::
  end()
  {
    std::cout << std::endl;
  }

  void
  ProgressBar::
  operator()(std::size_t pos)
  {
    print_bar(pos);
  }
}
