#include "mlib/geo/time_converter.h"

namespace mlib
{
  TimeConverter::
  TimeConverter(double secs_, int mins_, int hours_, int day_, int month_,
                int year_)
    :
    secs(secs_),
    mins(mins_),
    hours(hours_),
    day(day_),
    month(month_),
    year(year_)
  {}

  double
  TimeConverter::
  getUTCWeek()
  {
    time_t rawtime;
    struct tm * timeinfosynth;
    // get current timeinfo and modify it to the user's choice
    time(&rawtime);

    timeinfosynth = localtime(&rawtime);
    timeinfosynth->tm_year = year - 1900;
    timeinfosynth->tm_mon  = month - 1;
    timeinfosynth->tm_mday = day;

    // call mktime: timeinfo->tm_wday will be set
    mktime(timeinfosynth);

    double timestampWeekSeconds = timeinfosynth->tm_wday * 86400 +
                                  (hours * 3600 + mins * 60 + secs);
    return timestampWeekSeconds;
  }
}
