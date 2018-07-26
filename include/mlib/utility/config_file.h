#ifndef _MLIB_CONFIG_FILE_H__
#define _MLIB_CONFIG_FILE_H__

#include "mlib/utility/string.h"
#include "mlib/utility/parser/ini.h"

#include <sstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <fstream>
#include <limits> //std::limits

/** \addtogroup utility
 *  @{
 */
namespace mlib
{

  /**
   * @brief ParsedParameters
   */

  /**
   *
   */
  class ParsedParameters: public INIHandler
  {
  public:
    /**
     *
     */
    ParsedParameters(const std::string& filename_in_,
                     const std::string& filename_out_="",
                     const bool save_on_exit_ = true);

    /**
     *
     */
    ~ParsedParameters();

    /**
     * [save description]
     */
    void save();

    /**
     *
     */
    template<typename T>
    T
    add_new_entry(const std::string& section,
                  const std::string& name,
                  const T& default_value,
                  const T& max_val = std::numeric_limits<T>::max(),
                  const T& min_val = std::numeric_limits<T>::min());
  private:
    /**
     *
     */
    std::string filename_in;

    /**
     *
     */
    std::string filename_out;

    /**
     *
     */
    bool save_on_exit;
  };

}

/** @}*/
#endif //_MLIB_CONFIG_FILE_H__
