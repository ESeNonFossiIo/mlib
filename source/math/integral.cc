#include "mlib/math/integral.h"

#include <assert.h>     /* assert */
#include <iostream>

namespace _mlib
{

  Quadrature::
  Quadrature() {};

  double
  Quadrature::
  compute_value(const std::function<double(double)>& f,
                const double& init,
                const double& end) const
  {
    double sum(0.0);
    for(size_t i = 0; i<nodes.size(); i++)
      {
        sum += weights[i] * (end - init) * f(init + nodes[i]*(end - init));
      }
    return sum;
  };

  MidpointMethod::
  MidpointMethod()
    :
    Quadrature()
  {
    weights.resize(1);
    weights[0] = 1.0;
    nodes.resize(1);
    nodes[0] = 1.0 / 2.0;
  };

  TrapezoidalRule::
  TrapezoidalRule()
    :
    Quadrature()
  {
    weights.resize(2);
    weights[0] = 0.5;
    weights[1] = 0.5;
    nodes.resize(2);
    nodes[0] = 0.0;
    nodes[1] = 1.0;
  };



  Integral::
  Integral(const Quadrature& quad_,
           const double step_)
    :
    quad(quad_),
    step(step_)
  {};

  double
  Integral::
  compute_integral(
    const std::function<double(double)>& f,
    const double& x) const
  {
    assert(step  > 0.0);

    double sum(0.0);

    double x0,x1;
    if(x > 0.0)
      {
        x0 = 0.0;
        x1 = x0;
        while(x0 < x && x1 != x)
          {
            x1 = x0 + step;
            if(x1>x)
              x1 = x;
            sum +=  quad.compute_value(f, x0, x1);;
            x0 = x1;
          };
      }
    else
      {
        x0 = x;
        x1 = x0;
        while(x0 < 0.0 && x1 != 0.0)
          {
            x1 = x0 + step;
            if(x1>0.0)
              x1 = 0.0;
            sum -=  quad.compute_value(f, x0, x1);;
            x0 = x1;
          };
      }
    return sum;
  };

  double
  Integral::
  operator()(
    const std::function<double(double)>& f,
    double x) const
  {
    return this->compute_integral(f,x);
  };

};
