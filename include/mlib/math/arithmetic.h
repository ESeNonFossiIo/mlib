#ifndef __m_ARITHMETIC_H
#define __m_ARITHMETIC_H

#include <vector>  // std::vector
#include <limits>  // std::numeric_limits
#include <cstddef> // std::size_t

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  /**
   * argmax of a std::vector
   * @param  x vectors of doubles
   * @return   argmax
   */
  size_t argmax(const std::vector<double>& x);

  /**
   * argmin of a std::vector
   * @param  x vectors of doubles
   * @return   argmin
   */
  size_t argmin(const std::vector<double>& x);


  /**
   * max of a std::vector
   * @param  x vectors of doubles
   * @return   argmax
   */
  double max(const std::vector<double>& x);

  /**
   * min of a std::vector
   * @param  x vectors of doubles
   * @return   argmin
   */
  double min(const std::vector<double>& x);


  /**
   * Normalize a std::vector in the range [0,1]
   */
  std::vector<double>
  normalize(const std::vector<double>& x);

  /**
   * Return a std::vector<size_t> with the digits of the
   * binary representation of the point.
   * @param  num decimal number
   * @return     digits
   */
  std::vector<size_t>
  decimal_to_binary(const std::size_t& num);

  /**
   * Return an unsinded int from a std::vector representi a binary number
   * @param  num [description]
   * @return     [description]
   */
  std::size_t
  binary_to_decimal(const std::vector<size_t>& num);

  /**
   * Pythagoras Theorem without overflow and underflow problems
   * @param  a [description]
   * @param  b [description]
   * @return   [description]
   */
  double
  pythagoras(const double& a, const double& b);
}
/** @}*/
#endif // __m_ARITHMETIC_H
