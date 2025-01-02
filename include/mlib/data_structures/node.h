#ifndef _MLIB_DATA_STRUCTURES_
#define _MLIB_DATA_STRUCTURES_

#include <memory>

/** \addtogroup data_structure
 *  @{
 */

namespace mlib
{

  template<typedef VALUE>
  class Node
  {
  public:
    Node() {};

    Node(std::shared_ptr<Node> right_, std::shared_ptr<Node> left_)
      :
      right(right_),
      left(left_) {};

  private:
    VALUE val;

    std::shared_ptr<Node> right;
    std::shared_ptr<Node> left;
  }

};


/** @}*/

#endif //_MLIB_DATA_STRUCTURES_
