#ifndef _NUMERIX_INI_MANAGER_FILE_H__
#define _NUMERIX_INI_MANAGER_FILE_H__

#include "numerix/utility/logger.h"
#include "numerix/utility/types.h"

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream> // std::stringstream
#include <string>
#include <vector>

/** \addtogroup utility
 *  @{
 */
namespace numerix {
/**
 * \brief INI file parser and configuration handler.
 *
 * Provides functionality to read, parse, and manage configuration data in INI format.
 * INI files are organized into sections with key-value pairs. This handler allows
 * retrieving values with type conversion and provides file I/O operations.
 */
class INIHandler {

public:
    /**
     * \brief Default constructor for an empty configuration handler.
     */
    INIHandler();

    /**
     * \brief Constructor that loads configuration from a file or string.
     *
     * \param conf_string Path to INI file or INI format string
     * \param path Whether conf_string is a file path (true) or inline content (false)
     */
    INIHandler(const std::string& conf_string, bool path = true);

    /**
     * \brief Retrieve a configuration value with optional type conversion.
     *
     * \tparam TYPE Data type for the value (default: std::string)
     * \param section Configuration section name
     * \param name Configuration entry name within the section
     * \param default_value Value to return if entry is not found
     * \return Configuration value converted to TYPE, or default_value if not found
     */
    template <typename TYPE = std::string>
    TYPE get_val(const std::string& section,
                 const std::string& name,
                 const TYPE& default_value = zero<TYPE>());

    /**
     * \brief Get all section names in the configuration.
     *
     * \return Vector of section names
     */
    std::vector<std::string> get_sections();

    /**
     * \brief Get all entry names within a specific section.
     *
     * \param section Section name
     * \return Vector of entry names in that section
     */
    std::vector<std::string> get_entries(const std::string& section);

    /**
     * \brief Save configuration to an output stream.
     *
     * \param outputFile Output stream to write INI format data to
     */
    void save(std::ostream& outputFile);

    /**
     * \brief Save configuration to a file.
     *
     * \param filename Path to output file
     */
    void save(const std::string& filename);

    /**
     * \brief Validate and check the loaded configuration.
     */
    void check();

    /**
     * \brief Create a logger instance for this configuration.
     *
     * \param filename_ Optional log file name (default: empty)
     * \param write_on_stdcout_ Whether to output to console (default: true)
     * \return Logger instance configured for this handler
     */
    Logger get_logger(const std::string& filename_ = "", const bool& write_on_stdcout_ = true);

    /**
     * \brief Add a configuration entry.
     *
     * \param section Target section name
     * \param entry Entry name
     * \param val Entry value as string
     */
    void add_entry(const std::string& section, const std::string& entry, const std::string& val);

    /**
     * \brief Remove a configuration entry.
     *
     * \param section Target section name
     * \param entry Entry name to remove
     */
    void rm_entry(const std::string& section, const std::string& entry);

    /**
     * \brief Access configuration section as a map.
     *
     * \param label Section name
     * \return Reference to the map of entries in that section
     */
    std::map<std::string, std::string>& operator[](const std::string& label);

private:
    /**
     * \brief Internal storage for configuration data.
     *
     * Structure: section -> (entry -> value)
     */
    std::map<std::string, std::map<std::string, std::string>> conf;

    /**
     * \brief Parse an input stream containing INI format data.
     *
     * \param is Input stream to parse
     */
    void process_stream(std::istream& is);
};

} // namespace numerix
/** @}*/
#endif //_NUMERIX_INI_MANAGER_FILE_H__
