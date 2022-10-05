#ifndef __INTEGRAL__H_
#define __INTEGRAL__H_

#include <vector>
#include <functional>

/** \addtogroup math
 *  @{
 */

namespace mlib
{
  class Quadrature
  {
  public:
    Quadrature();

    Quadrature(const Quadrature& quad_)
      :
      nodes(quad_.nodes),
      weights(quad_.weights)
    {}

    double
    compute_value(const std::function<double(double)>& f,
                  const double& init,
                  const double& end) const;

  protected:
    std::vector<double> nodes;
    std::vector<double> weights;
  };

  class MidpointMethod : public Quadrature
  {
  public:
    MidpointMethod();
  };

  class TrapezoidalRule : public Quadrature
  {
  public:
    TrapezoidalRule();
  };

  class Integral
  {
  public:

    Integral(const Quadrature& quad_, const double step_);

    Integral(const Integral& integral_)
      :
      step(integral_.step),
      quad(integral_.quad)
    {}

    double
    compute_integral(
      const std::function<double(double)>& f,
      const double& x) const;

    double
    operator()(
      const std::function<double(double)>& f,
      double x) const;

    double  step;
    Quadrature quad;
  };

}

/** @}*/
#endif //__INTEGRAL__H_
