#ifndef _MLIB_HUFFMAN_
#define _MLIB_HUFFMAN_

#include <map>

/** \addtogroup coding
 *  @{
 */

namespace mlib
{

  class HuffmanCoding
  {
  public:
    HuffmanCoding(const std::string& text_)
      :
      text(text_)
    {
      for(std::string::iterator it = text.begin(); it != text.end(); ++it)
        {
          std::map<char, int>::iterator it2 = counter.find(*it);
          if(counter.end() != it2)
            {
              counter.at(*it) += 1;
            }
          else
            {
              counter.insert(std::make_pair(*it, 1));
            }
        }


    };

    void print_counter()
    {
      for(auto it = counter.begin(); it != counter.end(); ++it)
        {
          std::cout << it->first << " = " << it->second << std::endl;
        }
    };

    // void sort()
    // {
    //   auto cmp = [](std::pair<K,V> const & a, std::pair<K,V> const & b)
    //   {
    //     return a.second != b.second?  a.second < b.second : a.first < b.first;
    //   };
    //   std::sort(items.begin(), items.end(), cmp);
    // }
  private:
    std::string text;
    std::map<char, int> counter;
  };
};


/** @}*/

#endif //_MLIB_HUFFMAN_
