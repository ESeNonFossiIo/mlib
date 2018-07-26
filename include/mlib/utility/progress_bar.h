#ifndef _MLIB_PROGRESS_BAR_H__
#define _MLIB_PROGRESS_BAR_H__

#include <iostream>
#include <string>
#include <iomanip>

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  class ProgressBar
  {
  public:
    ProgressBar(
      unsigned int length_ = 60,
      std::string c_ = "=");


    void print_bar(unsigned int pos);

    void end();

    void operator()(unsigned int pos);

  private:
    std::string c;
    unsigned int length;
  };
}
/** @}*/
#endif // _MLIB_PROGRESS_BAR_H__
