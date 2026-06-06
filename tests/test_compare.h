#include "numerix/math/point.h"
#include <string>

bool are_equal(const numerix::Point& p,
               const numerix::Point& q,
               const double& tolerance = VAR_NUMERIX_ZERO_TOLERANCE)
{
    return (p - q).l_2_norm() < tolerance;
}

std::string are_equal(const double& val, const double& tolerance = VAR_NUMERIX_ZERO_TOLERANCE)
{
    return (std::abs(val) < tolerance ? "[OK]" : "[Fail]");
}
