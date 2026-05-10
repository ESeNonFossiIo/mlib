#ifndef _MLIB_PROGRESS_BAR_H__
#define _MLIB_PROGRESS_BAR_H__

#include <iomanip>
#include <iostream>
#include <string>

/** \addtogroup utility
 *  @{
 */
namespace mlib {
class ProgressBar {
public:
    ProgressBar(std::size_t length_ = 60, std::string c_ = "=");

    void print_bar(std::size_t pos);

    void end();

    void operator()(std::size_t pos);

private:
    std::string c;
    std::size_t length;
};
} // namespace mlib
/** @}*/
#endif // _MLIB_PROGRESS_BAR_H__
