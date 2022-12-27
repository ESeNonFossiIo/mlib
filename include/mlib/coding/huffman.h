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
      std::map<char, int> counter_tmp;

      for(std::string::iterator it = text.begin(); it != text.end(); ++it)
        {
          std::map<char, int>::iterator it2 = counter_tmp.find(*it);
          if(counter_tmp.end() != it2)
            {
              counter_tmp.at(*it) += 1;
            }
          else
            {
              counter_tmp.insert(std::make_pair(*it, 1));
            }
        }

      std::copy(counter_tmp.begin(),
                counter_tmp.end(),
                std::back_inserter<std::vector< std::pair<char, int>>>(counter));


      auto cmp = [=](
                   const std::pair<char, int>&   a,
                   const std::pair<char, int>&   b)
      {
        return a.second <  b.second;
      };
      std::sort(counter.begin(), counter.end(), cmp);

    };

    void print_counter()
    {
      for(auto it = counter.begin(); it != counter.end(); ++it)
        {
          std::cout << it->first << " = " << it->second << std::endl;
        }
    };
  private:
    std::string text;

    // Use this syntax for sorting the elemets accortding to the value.
    //  std::map does not allow to sort elements by value. It uses keys.
    std::vector< std::pair<char, int>> counter;
  };
};


/** @}*/

#endif //_MLIB_HUFFMAN_
