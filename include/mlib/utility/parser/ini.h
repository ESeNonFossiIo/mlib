#ifndef _MLIB_INI_MANAGER_FILE_H__
#define _MLIB_INI_MANAGER_FILE_H__

#include "mlib/utility/types.h"
#include "mlib/utility/logger.h"

#include <sstream>      // std::stringstream
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <fstream>
#include <cstddef>

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  class INIHandler
  {

  public:
    /**
     *
     */
    INIHandler();

    /**
     *
     */
    INIHandler(
      const std::string& conf_string,
      bool path=true);

    /**
     * [get_val description]
     * @param  section [description]
     * @param  name    [description]
     * @return         [description]
     */
    template<typename TYPE = std::string>
    TYPE get_val(
      const std::string& section,
      const std::string& name,
      const TYPE& default_value = zero<TYPE>());

    /**
     *
     */
    std::vector<std::string> get_sections();

    /**
     *
     */
    std::vector<std::string> get_entries(
      const std::string& section);


    /**
     * [save description]
     * @param outputFile [description]
     */
    void save(std::ostream& outputFile);

    /**
     * [save description]
     * @param filename [description]
     */
    void save(const std::string& filename);

    /**
     * [check description]
     */
    void check();

    /**
     * [get_log]
     */
    Logger
    get_logger(const std::string& filename_ = "",
               const bool& write_on_stdcout_ = true);

    /**
     * [add_entry description]
     * @param section [description]
     * @param entry   [description]
     * @param val     [description]
     */
    void add_entry(
      const std::string& section,
      const std::string& entry,
      const std::string& val);

    /**
     * [rm_entry description]
     * @param section [description]
     * @param entry   [description]
     */
    void rm_entry(
      const std::string& section,
      const std::string& entry);

    /**
     * [operator[] description]
     * @param  i [description]
     * @return   [description]
     */
    std::map<std::string, std::string>&
    operator[](const std::string& label);


  private:
    /**
     *
     */
    std::map<std::string, std::map<std::string, std::string>>
                                                           conf;

    /**
     * [process_stream description]
     * @param is [description]
     */
    void process_stream(std::istream& is);

  };

}
/** @}*/
#endif //_MLIB_INI_MANAGER_FILE_H__
