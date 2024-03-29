#ifndef _MLIB_FILE_H__
#define _MLIB_FILE_H__

#include <iostream>
#include <string>
#include <map>

/** \addtogroup utility
 *  @{
 */
namespace mlib
{

  /**
   * @brief Return a std::map containing dir, name, and extension of the file.
   */
  std::map<std::string, std::string>
  get_ext_and_name(const std::string& path);


  /**
   * @brief Return a bool that states the existency of a file.
   */
  bool
  file_exists(const std::string& path);


  /**
   * Return the number of lines that compose a file
   * @param  in_filename [description]
   * @return             [description]
   */
  std::size_t
  get_number_of_lines(const std::string& in_filename);
}

/** @}*/
#endif //__FILE__
