#include "mlib/math/convolution.h"
#include "mlib/math/constants.h"
#include <assert.h>     /* assert */

namespace mlib
{

  std::vector<double>
  convolve(std::vector<double>& in,
           int sizeh,
           double sigma)
  {
    double mu = 0;
    auto gauss = [&mu, &sigma](double x)
    {
      return std::exp(-1.0* (x-mu) * (x-mu) / (2 * sigma *
                                               sigma)) / (std::sqrt(2 * M_PI * sigma * sigma));
    };

    std::vector<double> ker;
    for(int i = -sizeh ; i < sizeh + 1; ++i)
      {
        ker.push_back(gauss(i));
      }

    std::vector<double> out(in);

    for(std::size_t i = sizeh; i < in.size() - sizeh - 1; ++i)
      {
        out[i] = 0.0;
        for(int j = -sizeh; j < sizeh + 1; ++j)
          out[i] += in[i - j] * ker[sizeh + j];
      }
    return out;
  }

  std::vector<double>
  remove_small_values(std::vector<double>& vec,
                      double toll_zero)
  {
    std::vector<double> v(vec);
    for(std::size_t i  = 0; i<v.size(); ++i)
      if(std::abs(v[i]) <toll_zero) v[i] = 0;
    return v;
  }

// Kernel
////////////////////////////////////////////////////////////////////////////////
  Kernel::
  Kernel(std::function<double(double)> function_,
         double support_min, double support_max)
    :
    function(function_)
  {
    support = std::make_pair(support_min, support_max);
  }

  std::function<double(double)>
  Kernel::
  get_kernel() const
  {
    return function;
  }

  double
  Kernel::
  operator()(double x) const
  {
    return function(x);
  }

  std::pair<int,int>
  Kernel::
  get_int_support() const
  {
    // assert first < second
    return std::make_pair((int) support.first,
                          (int) support.second);
  }

// GaussKernel
////////////////////////////////////////////////////////////////////////////////

  GaussKernel::
  GaussKernel(double sigma_,
              double mu_)
    :
    Kernel(),
    sigma(sigma_),
    mu(mu_)
  {
    // TODO: assert sigma > 0
    this->support = std::make_pair(-2*sigma, 2*sigma);
    this->function = [&](double x)
    {
      return std::exp(-1.0* (x-mu) * (x-mu) / (2 * sigma *
                                               sigma)) / (std::sqrt(2 * M_PI * sigma * sigma));
    };
  }

// Convolution
////////////////////////////////////////////////////////////////////////////////

  Convolution::
  Convolution(Kernel kernel_)
    :
    kernel(kernel_)
  {}

  std::vector<double>
  Convolution::
  compute(const std::vector<double>& in) const
  {
    std::vector<double> ker;
    std::pair<std::size_t, std::size_t > domain = kernel.get_int_support();
    for(std::size_t   i = domain.first;
        i <= domain.second;
        ++i)
      {
        // TODO: overload di ()...
        ker.push_back(kernel.get_kernel()(static_cast<double>(i)));
      }

    std::vector<double> out(in);
    for(std::size_t  i = domain.first;
        i < in.size() - domain.second  - 1;
        ++i)
      {
        out[i] = 0.0;
        for(std::size_t  j = domain.first;
            j <= domain.second;
            ++j)
          {
            assert(i >= j);
            out[i] += in[i - j] * ker[j];
          }
      }
    return out;
  }

  std::vector<double>
  Convolution::
  operator()(const std::vector<double>& v) const
  {
    return compute(v);
  } // for const objects

}
