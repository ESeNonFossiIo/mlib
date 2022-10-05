#include "mlib/utility/parser/ini.h"
#include "mlib/utility/string.h"

#include <algorithm>

namespace mlib
{

  INIHandler::
  INIHandler() {}

  INIHandler::
  INIHandler(const std::string& conf_string, bool path)
  {
    if(path)
      {
        std::ifstream infile(conf_string);
        process_stream(infile);
      }
    else
      {
        std::istringstream f(conf_string);
        process_stream(f);
      }
  }

  template<typename TYPE>
  TYPE
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const TYPE& /* FIX: default_value */)
  {
    return conf[section][name];
  }

  template<>
  std::string
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const std::string& default_value)
  {
    if(conf.count(section))
      return conf[section][name];
    else
      return default_value;
  }

  template<>
  double
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const double& default_value)
  {
    if(conf.count(section))
      return from_str_to_double(conf[section][name]);
    else
      return default_value;
  }

  template<>
  bool
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const bool& default_value)
  {
    if(conf.count(section))
      return from_str_to_bool(conf[section][name]);
    else
      return default_value;
  }

  template<>
  std::size_t
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const std::size_t& default_value)
  {
    if(conf.count(section))
      return from_str_to_unsigned_int(conf[section][name]);
    else
      return default_value;
  }

  template<>
  int
  INIHandler::
  get_val(
    const std::string& section,
    const std::string& name,
    const int& default_value)
  {
    if(conf.count(section))
      return from_str_to_int(conf[section][name]);
    else
      return default_value;
  }

  std::vector<std::string>
  INIHandler::
  get_sections()
  {
    std::vector<std::string> sections;
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           it_conf  = conf.begin();
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           end_conf = conf.end();

    for(; it_conf != end_conf; ++it_conf)
      {
        sections.push_back(it_conf->first);
      }

    return sections;
  }


  std::vector<std::string>
  INIHandler::
  get_entries(
    const std::string& section)
  {
    std::vector<std::string> entries;
    std::map<std::string, std::string>::iterator it_entry  =
      conf[section].begin();
    std::map<std::string, std::string>::iterator end_entry  =
      conf[section].end();

    for(; it_entry != end_entry; ++it_entry)
      {
        entries.push_back(it_entry->first);
      }

    return entries;
  }


  void
  INIHandler::
  save(std::ostream& outputFile)
  {
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           it_conf  = conf.begin();
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           end_conf = conf.end();

    for(; it_conf != end_conf; ++it_conf)
      {
        outputFile << "[" << it_conf->first << "]" <<
                   std::endl;
        std::map<std::string, std::string>::iterator it_entry  =
          it_conf->second.begin();
        std::map<std::string, std::string>::iterator end_entry  =
          it_conf->second.end();
        for(; it_entry != end_entry; ++it_entry)
          {
            outputFile << "     " << it_entry->first
                       << " =   " << it_entry->second <<
                       std::endl;
          }
      }
  }

  void
  INIHandler::
  save(const std::string& filename)
  {
    std::ofstream outputFile(filename);
    save(outputFile);
  }

  void
  INIHandler::
  check()
  {

    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           it_conf  = conf.begin();
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           end_conf = conf.end();

    for(; it_conf != end_conf; ++it_conf)
      {
        std::cout << "section ..... " << it_conf->first <<
                  std::endl;
        std::map<std::string, std::string>::iterator it_entry  =
          it_conf->second.begin();
        std::map<std::string, std::string>::iterator end_entry  =
          it_conf->second.end();
        for(; it_entry != end_entry; ++it_entry)
          {
            std::cout << "  ->  entry : " << it_entry->first <<
                      std::endl;
            std::cout << "        val : " << it_entry->second <<
                      std::endl;
          }
      }
  }

  Logger
  INIHandler::
  get_logger(const std::string& filename_,
             const bool& write_on_stdcout_)
  {
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           it_conf  = conf.begin();
    std::map<std::string, std::map<std::string, std::string>>::iterator
                                                           end_conf = conf.end();

    Logger return_log(filename_, write_on_stdcout_);
    for(; it_conf != end_conf; ++it_conf)
      {
        return_log.write("SECTION", it_conf->first, Color::yellow);
        std::map<std::string, std::string>::iterator it_entry  =
          it_conf->second.begin();
        std::map<std::string, std::string>::iterator end_entry  =
          it_conf->second.end();
        for(; it_entry != end_entry; ++it_entry)
          {
            return_log.value(it_entry->first,  it_entry->second);
          }
      }
    return return_log;
  }

  void
  INIHandler::
  rm_entry(const std::string& section,
           const std::string& entry)
  {
    conf[section].erase(entry);
    if(conf[section].size()==0)
      {
        conf.erase(section);
      }
  }

  void
  INIHandler::
  add_entry(const std::string& section,
            const std::string& entry,
            const std::string& val)
  {
    conf[section][entry] = val;
  }

  void
  INIHandler::
  process_stream(std::istream& is)
  {
    std::string section(""), entry, val;
    std::string line;
    while(std::getline(is, line))
      {
        std::string line_in(line);
        remove_if(line_in.begin(), line_in.end(), ::isspace);

        if(line_in.size() > 0)
          if(line_in.at(0)==';')
            continue;

        size_t equals       = line.find("=");
        size_t open_braket  = line.find("[") + 1;
        size_t close_braket = line.find_last_of("]");

        if(open_braket > 0
           && open_braket < close_braket)
          {
            section = line.substr(open_braket,
                                  close_braket-open_braket);
            remove_white_spaces(section);
            entry = "";
          }
        else
          {
            if(equals!=line.npos)
              {
                entry = line.substr(0, equals);
                remove_white_spaces(entry);
                val   = line.substr(equals+1, line.size() - equals);
                conf[section][entry] = val;
              }
            else
              {
                if(entry!="" && line!="")
                  conf[section][entry] += "\n"+line;
              }
          }
        ;
      }
  }

  std::map<std::string, std::string>&
  INIHandler::
  operator[](const std::string& label)
  {
    return conf[label];
  }
}
