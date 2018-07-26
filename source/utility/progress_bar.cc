#include "mlib/utility/progress_bar.h"

namespace mlib
{

  ProgressBar::
  ProgressBar(
    unsigned int length_,
    std::string c_)
    :
    length(length_),
    c(c_)
  {}

  void
  ProgressBar::
  print_bar(unsigned int pos)
  {
    unsigned int status = (int)((double) pos*
                                (double) length/100.0);

    std::cout << "\r [";
    for(unsigned int i = 0; i<length; ++i)
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
  operator()(unsigned int pos)
  {
    print_bar(pos);
  }
}
