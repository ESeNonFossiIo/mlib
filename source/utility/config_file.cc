#include "mlib/utility/config_file.h"

namespace mlib
{

// ParsedParameters
////////////////////////////////////////////////////////////////////////////////

  ParsedParameters::
  ParsedParameters(const std::string& filename_in_,
                   const std::string& filename_out_,
                   const bool save_on_exit_)
    :
    filename_in(filename_in_),
    filename_out(filename_out_==""?filename_in_:filename_out_),
    INIHandler(filename_in_),
    save_on_exit(save_on_exit_)
  {}

  ParsedParameters::
  ~ParsedParameters() {}

  void
  ParsedParameters::
  save()
  {
    if(save_on_exit)
      INIHandler::save(filename_out);
  }

  template<>
  double
  ParsedParameters::
  add_new_entry(const std::string& section,
                const std::string& name,
                const double& default_value,
                const double& max_val,
                const double& min_val)
  {
    std::string val = this->get_val<std::string>(section, name);
    if(val=="")
      this->add_entry(section, name, std::to_string(default_value));

    double num = from_str_to_double(val);

    if(num < min_val || num > max_val)
      return default_value;
    return num;
  }

  template<>
  unsigned int
  ParsedParameters::
  add_new_entry(const std::string& section,
                const std::string& name,
                const unsigned int& default_value,
                const unsigned int& max_val,
                const unsigned int& min_val)
  {
    std::string val = this->get_val<std::string>(section, name);
    if(val=="")
      this->add_entry(section, name, std::to_string(default_value));

    unsigned int num = from_str_to_unsigned_int(val);

    if(num <min_val || num> max_val)
      return default_value;
    return num;
  }

  template<>
  int
  ParsedParameters::
  add_new_entry(const std::string& section,
                const std::string& name,
                const int& default_value,
                const int& max_val,
                const int& min_val)
  {
    std::string val = this->get_val<std::string>(section, name);
    if(val=="")
      this->add_entry(section, name, std::to_string(default_value));

    int num = from_str_to_int(val);

    if(num < min_val || num > max_val)
      return default_value;
    return num;
  }

  template<>
  std::string
  ParsedParameters::
  add_new_entry(const std::string& section,
                const std::string& name,
                const std::string& default_value,
                const std::string&,
                const std::string&)
  {
    std::string val = this->get_val<std::string>(section, name);
    if(val=="")
      this->add_entry(section, name, default_value);

    return val;
  }


  template<>
  bool
  ParsedParameters::
  add_new_entry(const std::string& section,
                const std::string& name,
                const bool& default_value,
                const bool& max_val,
                const bool& min_val)
  {
    std::string val = this->get_val<std::string>(section, name);
    if(val=="")
      this->add_entry(section, name, std::to_string(default_value));

    return from_str_to_bool(val);
  }
}
