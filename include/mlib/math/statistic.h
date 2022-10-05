#ifndef __m_STATISTIC_H__
#define __m_STATISTIC_H__

#include <numeric>
#include <vector>
#include <cmath>

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  /**
   * Compute the n-moment of the vector v.
   * @param  v       Random variable
   * @param  n       Order of the momentum
   * @param  central
   * @return         n-moment of the vector v
   */
  double
  moment(
    const std::vector<double>& v,
    const std::size_t& n = 1,
    const bool& central = false);

  /**
   * @param  v Random variable
   * @return   Mean of v
   */
  double
  mean(const std::vector<double>& v);

  /**
   * @param  v Random variable
   * @return   Variance of v
   */
  double
  var(const std::vector<double>& v);

  /**
   * @param  v Random variable
   * @return   Standard deviation of v
   */
  double
  stddev(const std::vector<double>& v);
}
/** @}*/
#endif // __m_STATISTIC_H__
