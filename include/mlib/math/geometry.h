#ifndef __GEOMETRY__
#define __GEOMETRY__

#include <vector>

using namespace std;

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  std::vector<unsigned int>
  rectify(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const double& mean_tolerance = 0.001,
    const unsigned int& min_n_zeroes = 20);

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  extract_linear_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const double& mean_tolerance = 0.0005,
                                                    const unsigned int& min_n_zeroes = 20);

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  extract_uniform_grid(
                                                    const std::vector<double>& x,
                                                    const unsigned int& step = 50);

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  merge_increasing_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const bool equal = false);

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  merge_decreasing_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const bool equal = false);

  std::vector<double>
  interpolate_straight_parts(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<std::pair<unsigned int, unsigned int>>&
    idx);

}
/** @}*/
#endif // __GEOMETRY__
