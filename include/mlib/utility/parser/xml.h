#ifndef _MLIB_XML_MANAGER_FILE_H__
#define _MLIB_XML_MANAGER_FILE_H__

#include <sstream>      // std::stringstream
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <fstream>
#include <memory>
#include <regex>

#include <mlib/utility/string.h>

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  struct XMLEntry
  {
    size_t init;
    size_t end;

    std::string label;
    bool is_header;

    std::string text;
    std::map<std::string, std::string> properties;
  };

  /**
   * process a line and extract: labels, properties, text
   * @method process_line
   * @param  s            [description]
   */
  XMLEntry
  process_XML_text(const std::string& str);

  class XMLHandler
  {
  public:
    /**
     *
     */
    XMLHandler();

    /**
     *
     */
    XMLHandler(std::string filename, bool is_file = true);

    /**
     *
     */
    void print(int indent = 0);


    /**
     *
     */
    void
    save(const std::string& filename);

    /**
     * [operator[] description]
     * @param  i [description]
     * @return   [description]
     */
    std::shared_ptr<XMLHandler>
    operator[](const std::string& s);

  private:
    /**
     *
     */
    std::map<std::string, std::string> xml;

    /**
     *
     */
    std::map<std::string, std::string> xml_stylesheet;

    /**
     *
     */
    std::string val_text;

    /**
     *
     */
    std::map<std::string, std::string> properties;

    /**
     *
     */
    std::map<std::string, std::shared_ptr<XMLHandler>> xml_entries;
  };

}
/** @}*/
#endif //_MLIB_XML_MANAGER_FILE_H__
