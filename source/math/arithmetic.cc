#include "mlib/math/arithmetic.h"

#include <cmath>
#include <iostream>

namespace _mlib
{

  size_t argmax(const std::vector<double>& x)
  {
    size_t i_max = 0;
    double x_max = std::numeric_limits<double>::min();
    for(size_t i = 0; i < x.size(); ++i)
      {
        if(x[i] > x_max)
          {
            i_max = i;
            x_max = x[i];
          }
      }
    return i_max;
  }

  size_t argmin(const std::vector<double>& x)
  {
    size_t i_min = 0;
    double x_min = std::numeric_limits<double>::max();
    for(size_t i = 0; i < x.size(); ++i)
      {
        if(x[i] < x_min)
          {
            i_min = i;
            x_min = x[i];
          }
      }
    return i_min;
  }

  double max(const std::vector<double>& x)
  {
    return x[argmax(x)];
  }

  double min(const std::vector<double>& x)
  {
    return x[argmin(x)];
  }

  std::vector<double>
  normalize(const std::vector<double>& x)
  {
    double x_max = x[argmax(x)];
    double x_min = x[argmin(x)];
    std::vector<double> x_new(x);
    for(size_t i = 0; i < x.size(); ++i)
      {
        x_new[i] = (x[i] - x_min)/(x_max - x_min);
      }
    return x_new;
  }

  std::vector<size_t>
  decimal_to_binary(const unsigned int& num)
  {
    std::vector<size_t> r;
    unsigned int res  = num;

    unsigned int e = int(std::log2(res));
    unsigned int p = std::pow(2,e);

    while(p > 0)
      {
        if(res/p > 0)
          {
            r.push_back(1);
            res -= p;
          }
        else
          r.push_back(0);
        p /= 2;
      }

    return r;
  }

  unsigned int
  binary_to_decimal(const   std::vector<size_t>& num)
  {
    if(num.size()==0)
      return 0;

    unsigned int result = 0;
    unsigned int e = num.size();
    unsigned int p = 1;
    for(unsigned int i = 0; i < num.size(); ++i)
      {
        result += num[num.size() - 1 - i] * p;
        p *= 2;
      }
    return result;
  }


  double
  pythagoras(const double& a, const double& b)
  {
    double absa=std::fabs(a);
    double absb=std::fabs(b);
    if(absa > absb)
      return absa*sqrt(1.0+(absb/absa)*(absb/absa));
    else
      return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+(absa/absb)*(absa/absb)));
  }
}
