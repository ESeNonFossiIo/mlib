#ifndef __CONVOLUTION__H_
#define __CONVOLUTION__H_

#include <functional>
#include <vector>
#include <cmath>

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  std::vector<double>
  convolve(std::vector<double>& in, int sizeh,
           double sigma = 1.0);

  std::vector<double>
  remove_small_values(std::vector<double>& vec,
                      double toll_zero = 0.002);

  class Kernel
  {
  public:
    Kernel() {};

    Kernel(std::function<double(double)> function_,
           double support_min,
           double support_max);

    double operator()(double x) const;

    std::function<double(double)> get_kernel() const;

    std::pair<int,int> get_int_support() const;

  protected:
    std::pair<double, double> support;
    std::function<double(double)> function;
  };

  class GaussKernel : public Kernel
  {
  public:
    GaussKernel(double sigma_ = 1.0, double mu_ = 0.0);

  private:
    const double sigma;
    const double mu;

  };

  class Convolution
  {
  public:
    Convolution(Kernel kernel_=GaussKernel());

    std::vector<double>
    compute(const std::vector<double>& in) const;

    // std::vector<double>& operator()(std::vector<double>& v);
    std::vector<double> operator()(const std::vector<double>& v)
    const;    // for const objects

  private:
    Kernel kernel;
  };
}
/** @}*/
#endif //__CONVOLUTION__H_
