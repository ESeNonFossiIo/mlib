#ifndef _NUMERIX_CONFIG_FILE_H__
#define _NUMERIX_CONFIG_FILE_H__

#include "numerix/utility/parser/ini.h"
#include "numerix/utility/string.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits> //std::limits
#include <map>
#include <sstream>
#include <string>
#include <vector>

/** \addtogroup utility
 *  @{
 */
namespace numerix {

/**
 * @brief ParsedParameters
 */

/**
 *
 */
class ParsedParameters : public INIHandler {
public:
    /**
     *
     */
    ParsedParameters(const std::string& filename_in_,
                     const std::string& filename_out_ = "",
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
    template <typename T>
    T add_new_entry(const std::string& section,
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

} // namespace numerix

/** @}*/
#endif //_NUMERIX_CONFIG_FILE_H__
