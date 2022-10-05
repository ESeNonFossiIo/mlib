#ifndef __MATH_UTILITY__
#define __MATH_UTILITY__

// For numeric_limits:
#include <limits>

// For std::vector:
#include <vector>

// For std::function
#include <functional>

// TODO: rimuovere dopo aver spostato le implementazioni nel .cpp
#include <cmath>

// per normalize_range
#include <functional>

/** \addtogroup math
 *  @{
 */

namespace mlib
{
  template <typename T>
  int
  sgn(
    T val
  );

  double
  truncate_decimals(
    double num,
    double size = 10.0
  );

  std::vector<double>
  truncate_decimals_vec(
    std::vector<double>& vec,
    double size
  );

  template <typename T>
  T
  truncate(
    T num,
    T min = std::numeric_limits<T>::min(),
    T max = std::numeric_limits<T>::max()
  );

  template <typename T>
  std::vector<T>
  truncate_vec(
    std::vector<T>& vec,
    T min = std::numeric_limits<T>::min(),
    T max = std::numeric_limits<T>::max()
  );

  /**
   * Calculate discrete derivative with fixed step
   * @param  vec  function to derive
   * @param  step
   * @param  left sense
   * @return      derivative
   */
  std::vector<double>
  difference(
    std::vector<double>& vec,
    double step = 1,
    bool left = true
  );

  /**
   * Calculate discrete derivative of a function y with respect to x
   * @param  y    function to derive
   * @param  x    independent variable to use to derive
   * @param  left sense
   * @return      derivative
   */
  std::vector<double>
  difference(
    std::vector<double>& y,
    std::vector<double>& x,
    bool left = true
  );

  std::vector<double>
  accumulate(
    std::vector<double>& vec,
    double step = 1,
    bool left = true
  );

  std::vector<double>
  force_mean(
    std::vector<double>& vec,
    unsigned int consecutive = 4,
    double toll = 0.1
  );

  std::vector<std::size_t>
  flat_part(
    std::vector<double>& vec,
    double toll,
    unsigned int min_num_zeroes = 0
  );

  std::vector<int>
  straight_part(
    std::vector<double>& vec,
    double toll,
    unsigned int min_num_zeroes = 0
  );

  std::vector<double>
  interpolate(
    std::vector<double>& vec,
    std::vector<int>& interpolation,
    double toll = 0.05
  );

  std::vector<double>
  apply_lambda(
    std::vector<double>& vec,
    std::function<double(double)> func
  );

  std::vector<double>
  remove_singularities(
    std::vector<double>& vec,
    double toll_zero = 1e-6,
    double toll_jump = 0.0,
    unsigned int item_before = 2,
    unsigned int item_after  = 2,
    unsigned int singularity_lenght = 1,
    bool left = true);


  template <typename T>
  std::function<T(T)>
  normalize_range(const T val1, const T val2);
}
/** @}*/
#endif // __MATH_UTILITY__
