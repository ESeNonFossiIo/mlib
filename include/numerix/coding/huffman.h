#ifndef _NUMERIX_HUFFMAN_
#define _NUMERIX_HUFFMAN_

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

/** \addtogroup coding
 *  @{
 */

namespace numerix {

/**
 * \brief Huffman coding utility class for character frequency analysis.
 *
 * This class analyzes the frequency of characters in a given string
 * and provides sorted frequency information for Huffman encoding purposes.
 * Characters are sorted by frequency in ascending order.
 */
class HuffmanCoding {
public:
    /**
     * \brief Constructor that analyzes character frequencies in the input text.
     *
     * Counts occurrences of each character and sorts them by frequency.
     * Note: std::map sorts by keys, so we use a vector for frequency-based sorting.
     *
     * \param text_ Input string to analyze for character frequencies
     */
    inline HuffmanCoding(const std::string& text_) : text(text_)
    {
        std::map<char, int> counter_tmp;

        for (std::string::iterator it = text.begin(); it != text.end(); ++it) {
            std::map<char, int>::iterator it2 = counter_tmp.find(*it);
            if (counter_tmp.end() != it2) {
                counter_tmp.at(*it) += 1;
            } else {
                counter_tmp.insert(std::make_pair(*it, 1));
            }
        }

        std::copy(counter_tmp.begin(),
                  counter_tmp.end(),
                  std::back_inserter<std::vector<std::pair<char, int>>>(counter));

        auto cmp = [=](const std::pair<char, int>& a, const std::pair<char, int>& b) {
            return a.second < b.second;
        };
        std::sort(counter.begin(), counter.end(), cmp);
    };

    /**
     * \brief Print character frequencies to standard output.
     *
     * Outputs each character and its frequency count in the format: "char = count"
     */
    inline void print_counter()
    {
        for (auto it = counter.begin(); it != counter.end(); ++it) {
            std::cout << it->first << " = " << it->second << std::endl;
        }
    };

private:
    std::string text; ///< Input text being analyzed

    /**
     * \brief Vector of character-frequency pairs sorted by frequency.
     *
     * Note: We use a vector instead of std::map because std::map sorts by keys,
     * but we need sorting by frequency values for Huffman encoding.
     */
    std::vector<std::pair<char, int>> counter;
};
}; // namespace numerix

/** @}*/

#endif //_NUMERIX_HUFFMAN_
