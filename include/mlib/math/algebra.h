#ifndef TDT_ALGEBRA_H
#define TDT_ALGEBRA_H

#include <initializer_list>  // std::initializer_list
#include <vector>  // std::vector
#include <ostream>  // std::ostream


/** \addtogroup math
 *  @{
 */

namespace mlib
{
  class Polynomial
  {
  public:
    /**
     * constructor
     */
    Polynomial(const std::initializer_list<double>& list);

    Polynomial(const std::vector<double>& v);

    Polynomial();


    double
    operator()(const double& x) const;

    size_t
    deg() const;

    size_t
    size() const;

    Polynomial
    d(const std::size_t& i = 1) const;

    double&
    operator[](size_t i);

    const double&
    operator[](size_t i) const;


    /**
     *
     */
    friend
    std::ostream&
    operator<< (std::ostream& output, const Polynomial& p);

  protected:
    std::vector<double> c;
  };

}
/** @}*/
#endif // TDT_ALGEBRA_H
