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

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  class XMLEntry
  {
  public:
    /**
     *
     */
    XMLEntry(const std::string& label_,
             const std::string& value_="");

    /**
     *
     */
    void add_property(const std::string& name,
                      const std::string& val);

    /**
     *
     */
    void add(XMLEntry entry);

    /**
     *
     */
    std::string append_string(unsigned int depth = 0);

    std::string&
    get_val();

    std::string&
    get_label();

    std::string&
    get_property(std::string& str);

    XMLEntry*
    get_element(const unsigned int& i);

    XMLEntry*
    operator[](const unsigned int& i);

  private:
    /**
     *
     */
    std::string value;

    /**
       *
       */
    std::string label;

    /**
     *
     */
    std::map<std::string, std::string> property;

    /**
     *
     */
    std::vector<std::shared_ptr<XMLEntry>> sub_list;
  };

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
    XMLHandler(std::string filename);

    /**
     *
     */
    void add(XMLEntry new_entry);

    /**
     *
     */
    void print();

    /**
     *
     */
    void set_style(std::string style_);


    /**
     *
     */
    void set_xml_header(std::string name, std::string val);

    /**
     *
     */
    void set_xml_stylesheet(std::string name, std::string val);


    /**
     *
     */
    std::string get_file(unsigned int depth = 0);

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
    XMLEntry*
    get_element(const unsigned int& i);

    /**
     * [operator[] description]
     * @param  i [description]
     * @return   [description]
     */
    XMLEntry*
    operator[](const unsigned int& i);

  private:
    /**
     *
     */
    std::vector<std::shared_ptr<XMLEntry>> xml_entries;

    /**
       *
       */
    std::map<std::string, std::string> xml_header;

    /**
     *
     */
    std::map<std::string, std::string> xml_stylesheet;
  };

}
/** @}*/
#endif //_MLIB_XML_MANAGER_FILE_H__
