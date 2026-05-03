#ifndef _MLIB_PCL_LOGGER
#define _MLIB_PCL_LOGGER

#include <fstream>
#include <ctime>
#include <vector>
#include <string>

#include "mlib/utility/color.h"

namespace mlib
{

  using namespace std;

  /**
   * \brief A logging utility class for recording timestamped messages.
   * 
   * The Logger class provides methods to write various types of messages with
   * different color formatting (warnings, errors, status updates, values, etc.).
   * Messages can be stored in memory and optionally written to a file or console.
   */
  class Logger
  {
  public:
    /**
     * \brief Constructor for creating a logger instance.
     * 
     * \param filename_ Optional filename for saving logs to file (default: empty)
     * \param write_on_stdcout_ Whether to write messages to standard output (default: false)
     */
    Logger(const std::string& filename_ = "",
           const bool& write_on_stdcout_ = false);

    /**
     * \brief Copy constructor.
     * 
     * \param copy Logger instance to copy from
     */
    Logger(const Logger& copy);

    /**
     * \brief Destructor that closes any open file handles.
     */
    ~Logger();

    /**
     * \brief Save all logged messages to a file.
     * 
     * \param filename_ Optional filename to override the one set in constructor
     */
    void
    save_on_file(const std::string& filename_ = "");

    /**
     * \brief Print all logged messages to standard output.
     */
    void
    print();

    /**
     * \brief Write a generic message with color formatting.
     * 
     * \param msg Message label/type
     * \param str Message content
     * \param color Color to use for formatting
     */
    void write(const std::string& msg,
               const std::string& str,
               const GeneralColor& color);

    /**
     * \brief Log a warning message (formatted in yellow).
     * 
     * \param str Warning message content
     */
    void warning(const std::string& str);

    /**
     * \brief Log an error message (formatted in red).
     * 
     * \param str Error message content
     */
    void error(const std::string& str);

    /**
     * \brief Log a status message (formatted in cyan).
     * 
     * \param str Status message content
     */
    void status(const std::string& str);

    /**
     * \brief Log a key-value pair message (formatted in green).
     * 
     * \param str Key name
     * \param val Value content
     */
    void value(const std::string& str, const std::string& val);

    /**
     * \brief Log a general message (formatted in white).
     * 
     * \param str Message content
     */
    void msg(const std::string& str);

  private:
    clock_t     begin;              ///< Clock time when logger was created
    std::string filename;           ///< Output filename for file logging
    ofstream    file;               ///< Output file stream
    bool        write_on_stdcout;   ///< Flag to enable console output
    bool        write_on_file;      ///< Flag to enable file output

    std::vector<std::string> content; ///< Vector of logged messages
  };


}
#endif //_MLIB_PCL_LOGGER
