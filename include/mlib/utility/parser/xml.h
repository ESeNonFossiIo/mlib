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
  process_XML_text(const std::string& str)
  {
    XMLEntry ret;

    std::size_t init(str.find('<')), end;
    ret.init      = init;
    ret.is_header = (str.at(init+1) == '?');

    std::size_t init_label = ret.is_header ? init + 2 : init + 1;
    std::size_t end_label =
      std::min(
        str.find('>', init + 1),
        str.find(' ', init + 1)
      );
    ret.label = str.substr(init_label, end_label - init_label);

    std::size_t init_first_tag = end_label;
    std::size_t end_first_tag  = str.find(ret.is_header ? '?' : '>', end_label);
    std::string properties_raw = str.substr(init_first_tag + 1,
                                            end_first_tag - 1 - init_first_tag);
    std::vector<std::string> properties =
      split(properties_raw,
            " ",
            false,
            false);

    for(auto it  = properties.begin(); it != properties.end(); it++)
      {
        std::vector<std::string> entry = split(*it,
                                               "=",
                                               false,
                                               true);
        std::size_t init_prop = entry[1].find("\"") + 1;
        std::size_t end_prop  = entry[1].find("\"", init_prop);
        ret.properties.insert(std::pair<std::string, std::string>(entry[0],
                                                                  entry[1].substr(init_prop,end_prop-init_prop)));
      }

    if(ret.is_header)
      {
        end = end_first_tag + 2;
      }
    else
      {
        size_t init_txt = end_first_tag + 1, end_txt;

        size_t counter_tag = 1;
        size_t next_t      = init_txt;

        while(counter_tag >= 1)
          {
            size_t open  = std::min(str.find("<"+ret.label, next_t), str.size());
            size_t close = std::min(str.find("</"+ret.label, next_t), str.size());

            if((0 < open) && (open < close))
              {
                counter_tag++;
                next_t = open + 1;
              }
            else
              {
                counter_tag--;
                next_t = close + 1;
              }
          }
        end_txt = next_t - 1;
        end = next_t + ret.label.size() + 3 ;

        ret.text = str.substr(init_txt, end_txt - init_txt);
      }
    ret.end = end;

    return ret;
  }

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
    void add(const std::string& label, std::shared_ptr<XMLHandler> new_entry);

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
    std::shared_ptr<XMLHandler>
    operator[](const std::string& s);

  private:
    /**
     *
     */
    std::map<std::string, std::shared_ptr<XMLHandler>> xml_entries;

    /**
     *
     */
    std::map<std::string, std::string> xml_stylesheet;
  };

}
/** @}*/
#endif //_MLIB_XML_MANAGER_FILE_H__
