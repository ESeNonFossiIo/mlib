#include "mlib/math/statistic.h"

namespace mlib
{

  double
  moment(
    const std::vector<double>& v,
    const std::size_t& n,
    const bool& central)
  {
    double sum = 0.0;
    double mu  = 0.0;

    if(central || n == 1)
      {
        for(auto x: v)
          mu += x;
        mu /= v.size();

        if(n==1)
          return mu;
      }

    for(auto x: v)
      {
        sum += std::pow(x - mu, n);
      }

    sum /= v.size();

    return sum;
  }

  double
  mean(const std::vector<double>& v)
  {
    return moment(v, 1);
  }

  double
  var(const std::vector<double>& v)
  {
    return moment(v, 2, true);
  }

  double
  stddev(const std::vector<double>& v)
  {
    return std::sqrt(var(v));
  }

}
