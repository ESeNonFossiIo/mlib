#ifndef  __m_TIME_CONVERTER_H__
#define  __m_TIME_CONVERTER_H__

#include <time.h>

/** \addtogroup geo
 *  @{
 */
namespace _mlib
{
  class TimeConverter
  {
  public:
    TimeConverter(double secs_, int mins_, int hours_, int day_, int month_,
                  int year_);;

    double getUTCWeek();

  private:
    double secs;
    int    mins;
    int    hours;
    int    day;
    int    month;
    int    year;
  };

}
/** @}*/
#endif // __m_TIME_CONVERTER_H__
