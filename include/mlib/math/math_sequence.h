#ifndef __m_MATH_SEQUENCE_H__
#define __m_MATH_SEQUENCE_H__

#include <iostream>     // std::cout
#include <functional>   // std::function

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
            const std::function<T(std::vector<T>, int)> next_element,
            const int n_elements = 10)
      :
      sequence(init_sequence)
    {
      sequence.resize(n_elements);
      for(size_t i = init_sequence.size(); i < n_elements; ++i)
        {
          auto tmp = std::vector<T>(sequence.begin()+i-init_sequence.size(),
                                    sequence.begin()+i);
          auto t = next_element(tmp, i);
          sequence[i] = t;
        };
    };
    ~MathSeq() {};

    typedef typename std::vector<T>::iterator       iterator;
    typedef typename std::vector<T>::const_iterator const_iterator;
    iterator begin()
    {
      return sequence.begin();
    }
    iterator end()
    {
      return sequence.end();
    }

  private:
    std::vector<T> sequence;
    int pattern_lenght;
  };

}
/** @}*/
#endif // __m_MATH_SEQUENCE_H__
