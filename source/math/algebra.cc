#include "mlib/math/algebra.h"

#include <cassert>  // std::assert

namespace mlib
{

  Polynomial::
  Polynomial(const std::initializer_list<double>& list)
    :
    c(list.size(), 0.0)
  {
    unsigned int i = 0;
    for(auto l: list)
      {
        c[i++] = l;
      }
  };

  Polynomial::
  Polynomial(const std::vector<double>& v)
    :
    c(v)
  {};

  Polynomial::
  Polynomial()
    :
    c(1, 0.0)
  {};


  double
  Polynomial::
  operator()(const double& x) const
  {
    double result(c[c.size() - 1]);
    for(unsigned int j = 1; j<c.size(); j++)
      {
        result *= x;
        result += c[c.size() - 1 - j];
      }
    return result;
  }

  size_t
  Polynomial::
  deg() const
  {
    return c.size() - 1;
  }

  size_t
  Polynomial::
  size() const
  {
    return c.size();
  }

  Polynomial
  Polynomial::
  d(const unsigned int& i) const
  {
    if(i==0)
      {
        return *this;
      }
    else if(i>this->deg() || this->deg() < 1)
      {
        return Polynomial();
      }
    else
      {
        std::vector<double> new_c(this->deg());
        for(unsigned int j = 0; j<new_c.size(); j++)
          {
            new_c[j] = (j+1)*c[j+1];
          }
        return Polynomial(new_c).d(i-1);
      }

  }

  double&
  Polynomial::
  operator[](size_t i)
  {
    return c[i];
  };

  const double&
  Polynomial::
  operator[](size_t i) const
  {
    return c[i];
  };


  std::ostream&
  operator<< (std::ostream& output, const Polynomial& p)
  {
    output << "[";
    for(unsigned int i = 0; i < p.size() - 1; ++i)
      output << p[i] << ",";
    output << p[p.deg()] << "]";
    return output;
  };

}
