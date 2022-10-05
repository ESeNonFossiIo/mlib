#include "mlib/math/math_sequence.h"

namespace mlib
{

  template<typename T>
  MathSeq<T>::
  MathSeq(const std::vector<T>& init_sequence,
          const std::function<T(std::vector<T>, int)>& next_element_,
          const int /* FIX: first_element */,
          const int n_elements)
    :
    sum(0),
    sequence(init_sequence),
    next_element(next_element_)
  {
    pattern_lenght = init_sequence.size();
    size           = init_sequence.size();
    for(auto e: init_sequence)
      sum += e;
    compute_elements(n_elements);
  }

  template<typename T>
  MathSeq<T>::
  ~MathSeq() {}

  template<typename T>
  void
  MathSeq<T>::
  compute_elements(const int n_elements)
  {
    if(static_cast<std::size_t>(n_elements) < sequence.size()) return;

    sequence.resize(n_elements);
    for(std::size_t i = size; i < static_cast<std::size_t>(n_elements); ++i)
      {
        auto tmp = std::vector<T>(sequence.begin() + i - pattern_lenght,
                                  sequence.begin() + i);
        auto t = next_element(tmp, i);
        sequence[i] = t;
        sum += t;
      }

    size = n_elements;
  }

  template<typename T>
  T
  MathSeq<T>::
  get_sum()
  {
    return sum;
  }

  template<typename T>
  typename std::vector<T>::iterator
  MathSeq<T>::begin()
  {
    return sequence.begin();
  }

  template<typename T>
  typename std::vector<T>::iterator
  MathSeq<T>::
  end()
  {
    return sequence.end();
  }

  template class MathSeq<int>;
  template class MathSeq<double>;
}
