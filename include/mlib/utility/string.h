#ifndef _MLIB_STRING_UTILITY__
#define _MLIB_STRING_UTILITY__

// per std::string
#include <string>

// per split
#include <vector>

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  /**
   * Convert in string to lower case
   * @param  in [description]
   * @return    [description]
   */
  std::string
  lower_case(const std::string& in);

  /**
   * Convert in string to upper case
   * @param  in [description]
   * @return    [description]
   */
  std::string
  upper_case(const std::string& in);

  /**
   * @brief from_str_to_double convert a std::string in a double.
   * @param in
   * @return a double
   */
  double
  from_str_to_double(const std::string in);

  /**
  * @brief from_double_to_str convert a double in a std::string.
  * @param in
  * @return a double
   */
  std::string
  from_double_to_str(double Number);

  /**
   * @brief from_str_to_unsigned_int convert a std::string in a unsigned int.
   * @param in
   * @return a unsigned int
   */
  unsigned int
  from_str_to_unsigned_int(const std::string in);

  /**
   * @brief from_str_to_int convert a std::string in a int.
   * @param in
   * @return a int
   */
  int
  from_str_to_int(const std::string in);

  /**
   * @brief from_str_to_bool convert a std::string in a bool.
   * @param in
   * @return a bool
   */
  bool
  from_str_to_bool(const std::string in);

  /**
   * @brief split
   * @param str
   * @param delim
   * @param remove_white_space remove white spaces in the field
   * @param allow_empty_fields remove empty fields from the csv
   * @return
   */
  std::vector<std::string>
  split(const std::string& str,
        const std::string& delim = ";",
        const bool remove_white_space = false,
        const bool allow_empty_fields = true);

  /**
   *
   */
  std::string
  hline(const unsigned int& size = 10,
        const std::string& c = "-");

  /**
   * Remove all white spaces from a string.
   * @param  str [description]
   * @return     [description]
   */
  std::string remove_white_spaces(std::string& str);

  /**
   * Remove all white spaces from a string, newline included.
   * @param  str [description]
   * @return     [description]
   */
  std::string trim(std::string& str);

  /**
   * Get all the content of a file and save it in a string.
   * @param  filename [description]
   * @return     [description]
   */
  std::string get_file_content(std::string& filename);

}
/** @}*/
#endif //_MLIB_STRING_UTILITY__
