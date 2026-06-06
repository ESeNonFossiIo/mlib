#ifndef _NUMERIX_PROGRESS_BAR_H__
#define _NUMERIX_PROGRESS_BAR_H__

#include <iomanip>
#include <iostream>
#include <string>

/** \addtogroup utility
 *  @{
 */
namespace numerix {
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
} // namespace numerix
/** @}*/
#endif // _NUMERIX_PROGRESS_BAR_H__
