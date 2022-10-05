#ifndef __GEOMETRY__
#define __GEOMETRY__

#include <vector>

using namespace std;

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  std::vector<std::size_t>
  rectify(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const double& mean_tolerance = 0.001,
    const std::size_t& min_n_zeroes = 20);

  std::vector<std::pair<std::size_t, std::size_t>>
                                                extract_linear_parts(
                                                  const std::vector<double>& x,
                                                  const std::vector<double>& y,
                                                  const double& mean_tolerance = 0.0005,
                                                  const std::size_t& min_n_zeroes = 20);

  std::vector<std::pair<std::size_t, std::size_t>>
                                                extract_uniform_grid(
                                                  const std::vector<double>& x,
                                                  const std::size_t& step = 50);

  std::vector<std::pair<std::size_t, std::size_t>>
                                                merge_increasing_parts(
                                                  const std::vector<double>& x,
                                                  const std::vector<double>& y,
                                                  const bool equal = false);

  std::vector<std::pair<std::size_t, std::size_t>>
                                                merge_decreasing_parts(
                                                  const std::vector<double>& x,
                                                  const std::vector<double>& y,
                                                  const bool equal = false);

  std::vector<double>
  interpolate_straight_parts(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<std::pair<std::size_t, std::size_t>>&
    idx);

}

/** @}*/
#endif // __GEOMETRY__
