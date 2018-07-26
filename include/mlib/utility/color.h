#ifndef __MYLIB_COLOR_UTILITY__
#define __MYLIB_COLOR_UTILITY__

// per std::string
#include <string>

/** \addtogroup utility
 *  @{
 */
namespace _mlib
{

  class GeneralColor
  {
  public:
    /**
     * [GeneralColor description]
     * @param color_ [description]
     * @param style_ [description]
     */
    GeneralColor(
      const int& color_ = 32,
      const int& style_ = 1);

    /**
     * [init description]
     * @return [description]
     */
    std::string init() const;

    /**
     * [end description]
     * @return [description]
     */
    std::string end() const;

  private:
    int color;
    int style;
  };


  namespace Color
  {
    static const GeneralColor black(30);
    static const GeneralColor red(31);
    static const GeneralColor green(32);
    static const GeneralColor yellow(33);
    static const GeneralColor blue(34);
    static const GeneralColor magenta(35);
    static const GeneralColor cyan(36);
    static const GeneralColor light_gray(37);
    static const GeneralColor white(97);
  }

}
/** @}*/
#endif //__MYLIB_COLOR_UTILITY__
