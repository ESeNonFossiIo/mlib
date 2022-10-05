#ifndef __m_MATH_SEQUENCE_H__
#define __m_MATH_SEQUENCE_H__

#include <functional>   // std::function
#include <vector>

/** \addtogroup math
 *  @{
 */
namespace mlib
{

  template<typename T>
  class MathSeq
  {
  public:
    MathSeq(const std::vector<T>& init_sequence,
            const std::function<T(std::vector<T>, int)>& next_element_,
            const int first_element = 0,
            const int n_elements = 10);;
    ~MathSeq();

    void compute_elements(const int n_elements = 10);

    T get_sum();

    typename std::vector<T>::iterator begin();
    typename std::vector<T>::iterator end();

  private:
    T sum;
    size_t size;
    std::vector<T> sequence;
    std::function<T(std::vector<T>, int)> next_element;
    int pattern_lenght;
  };

}
/** @}*/
#endif // __m_MATH_SEQUENCE_H__
