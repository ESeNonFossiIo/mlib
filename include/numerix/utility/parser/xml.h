#ifndef _NUMERIX_XML_MANAGER_FILE_H__
#define _NUMERIX_XML_MANAGER_FILE_H__

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <sstream> // std::stringstream
#include <string>
#include <vector>

#include <numerix/utility/string.h>

/** \addtogroup utility
 *  @{
 */
namespace numerix {
struct XMLEntry {
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
XMLEntry process_XML_text(const std::string& str);

class XMLHandler {
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
    void save(const std::string& filename);

    /**
     * [operator[] description]
     * @param  s [description]
     * @return   [description]
     */
    XMLHandler operator[](const std::string& s);

    /**
     * [operator[] description]
     * @return   [description]
     */
    std::string operator()();

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
    std::map<std::string, XMLHandler> xml_entries;
};

} // namespace numerix
/** @}*/
#endif //_NUMERIX_XML_MANAGER_FILE_H__
