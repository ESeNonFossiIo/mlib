#include <algorithm>

#include "mlib/utility/parser/xml.h"
#include "mlib/utility/string.h"

namespace mlib
{

  XMLEntry::
  XMLEntry(const std::string& label_,
           const std::string& value_)
    :
    value(value_),
    label(label_)
  {}

  void
  XMLEntry::
  add_property(const std::string& name,
               const std::string& val)
  {
    property[name] = val;
  }

  void
  XMLEntry::
  add(XMLEntry entry)
  {
    sub_list.push_back(std::make_shared<XMLEntry>(entry));
  }

  std::string
  XMLEntry::
  append_string(unsigned int depth)
  {
    std::string ss;

    for(unsigned int i = 0; i<depth; ++i)
      ss += "  ";
    ss += "<" + label;

    std::map<std::string, std::string>::iterator it = property.begin();
    std::map<std::string, std::string>::iterator end = property.end();
    for(; it!=end; ++it)
      {
        ss +=  " " + it->first + "=\"" + it->second + "\"";
      }
    ss += ">";

    std::string no_space(value);
    if(remove_white_spaces(value)!="")
      {
        ss +="\n";

        std::string key = "\n";
        size_t found = value.rfind(key);
        if(found!=std::string::npos)
          value.replace(found,key.length(),"");
        found = value.rfind(key);
        if(found!=std::string::npos)
          value.replace(found,key.length(),"");

        for(unsigned int i = 0; i<depth+1; ++i)
          ss += "  ";
        ss += value;
        ss +="\n";
      }
    else
      {
        if(sub_list.size()>0)
          ss +="\n";
      }

    std::vector<std::shared_ptr<XMLEntry>>::iterator itl = sub_list.begin();
    std::vector<std::shared_ptr<XMLEntry>>::iterator endl = sub_list.end();
    for(; itl!=endl; ++itl)
      {
        ss += itl->get()->append_string(depth+1);
        ss +="\n";
      }

    for(unsigned int i = 0; i<depth; ++i)
      ss += "  ";
    ss += "</" + label + ">";

    return ss;
  }


  std::string&
  XMLEntry::
  get_val()
  {
    return value;
  }

  std::string&
  XMLEntry::
  get_label()
  {
    return label;
  }

  std::string&
  XMLEntry::
  get_property(std::string& str)
  {
    return property[str];
  }

  XMLEntry*
  XMLEntry::
  get_element(const unsigned int& i)
  {
    return sub_list[i].get();
  }

  XMLEntry*
  XMLEntry::
  operator[](const unsigned int& i)
  {
    return get_element(i);
  }

  XMLHandler::
  XMLHandler() {}

  std::map<std::string, std::string>
  get_property(std::string& text)
  {
    std::map<std::string, std::string> result;
    std::replace(
      text.begin(),
      text.end(),
      '\"',  '\'');

    std::replace(
      text.begin(),
      text.end(),
      '=',  ' ');

    auto parts = split(text, "\' ");
    for(unsigned int i = 0; i<parts.size(); ++i)
      {
        std::vector<std::string>  name_and_val = split(parts[i], "\'");
        if(name_and_val[0]!="" && name_and_val.size()>1)
          result[remove_white_spaces(name_and_val[0])] = name_and_val[1];
      }
    return result;
  }

  std::map<std::string, std::string>
  process_meta_tag(const std::string& meta_tag,
                   std::string& text)
  {
    std::string xml_stylesheet_property_raw = "";
    std::map<std::string, std::string> result;
    if(text.find(meta_tag) > 0)
      {
        std::string label = meta_tag;
        unsigned int init = text.find(meta_tag) + label.size() + 1;
        unsigned int end  = text.find("?>", init) - 1;
        xml_stylesheet_property_raw = text.substr(init, end - init);
        text.erase(text.find(meta_tag), text.find("?>",
                                                  init) - text.find(meta_tag) + 2);
      }
    return get_property(xml_stylesheet_property_raw);
  }

  XMLEntry
  get_entry(std::string& text)
  {
    unsigned int pos      = text.find("<");
    unsigned int start    = pos;
    unsigned int with     = 0;
    unsigned int without  = 1;

    while(with != without)
      {
        unsigned int next = text.find("<", pos+1);
        if(text.at(next+1) == '/')
          {
            with ++;
          }
        else
          {
            without ++;
          }
        pos = next;
      }

    unsigned int close = pos;
    unsigned int end = text.find(">", close+1);

    unsigned int label_init = text.find_first_not_of(" ", start) +1;
    unsigned int label_end =
      std::min(
        text.find(" ", label_init),
        text.find(">", label_init)
      );
    std::string label = text.substr(label_init, label_end - label_init);

    std::string properties_raw = text.substr(label_end, text.find(">",
                                                                  label_end)-label_end);
    std::map<std::string, std::string> properties = get_property(properties_raw);

    std::string txt = text.substr(
                        text.find(">", label_end) + 1,
                        text.find("<", label_end) - text.find(">", label_end) -1);

    XMLEntry entry(label, txt);
    auto it = properties.begin();
    auto end_iterator = properties.end();
    for(; it!=end_iterator; ++it)
      entry.add_property(it->first, it->second);

    std::string next_text =  text.substr(text.find("<", label_end),
                                         close - text.find("<", label_end));
    while(next_text.find("<")<next_text.size())
      {
        XMLEntry sub_entry = get_entry(next_text);
        entry.add(sub_entry);
      };
    text.erase(start, end-start +1);

    return entry;
  }

  XMLHandler::
  XMLHandler(std::string filename)
  {

    std::ifstream infile(filename);
    std::string line;
    std::string text = "";
    while(std::getline(infile, line))
      {
        text += " " + line;
      }

    xml_header =
      process_meta_tag("<?xml", text);

    xml_stylesheet =
      process_meta_tag("<?xml-stylesheet", text);

    while(text.find("<") < text.size())
      {
        xml_entries.push_back(std::make_shared<XMLEntry>(get_entry(text)));
      }

  }

  void
  XMLHandler::
  set_style(std::string style_)
  {
    xml_stylesheet["href"] = style_;
  }

  void
  XMLHandler::
  add(XMLEntry new_entry)
  {
    xml_entries.push_back(std::make_shared<XMLEntry>(new_entry));
  }

  void
  XMLHandler::
  print()
  {
    std::cout << get_file() << std::endl;
  }

  std::string
  XMLHandler::
  get_file(unsigned int depth)
  {
    std::string text;

    {
      text += "<?xml ";
      std::map<std::string, std::string>::iterator itl  = xml_header.begin();
      std::map<std::string, std::string>::iterator endl = xml_header.end();
      for(; itl!=endl; ++itl)
        {
          text +=  itl->first + "=\'" + itl->second + "\' ";
        }
      text += "?> \n";
    }
    if(xml_stylesheet["href"]!="")
      {
        text += "<?xml-stylesheet ";
        std::map<std::string, std::string>::iterator itl  = xml_stylesheet.begin();
        std::map<std::string, std::string>::iterator endl = xml_stylesheet.end();
        for(; itl!=endl; ++itl)
          {
            text +=  itl->first + "=\'" + itl->second + "\' ";
          }
        text += "?> \n";
      }

    {
      std::vector<std::shared_ptr<XMLEntry>>::iterator itl  = xml_entries.begin();
      std::vector<std::shared_ptr<XMLEntry>>::iterator endl = xml_entries.end();
      for(; itl!=endl; ++itl)
        {
          text +=  itl->get()->append_string();
        }
    }

    return text;
  }

  void
  XMLHandler::
  save(const std::string& filename)
  {
    std::ofstream outputFile(filename);
    outputFile << get_file();
  }

  void
  XMLHandler::
  set_xml_header(std::string name, std::string val)
  {
    xml_header[name] = val;
  }

  void
  XMLHandler::
  set_xml_stylesheet(std::string name, std::string val)
  {
    xml_stylesheet[name] = val;
  }

  XMLEntry*
  XMLHandler::
  get_element(const unsigned int& i)
  {
    return xml_entries[i].get();
  }

  XMLEntry*
  XMLHandler::
  operator[](const unsigned int& i)
  {
    return get_element(i);
  }
}
