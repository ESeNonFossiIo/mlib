#include <algorithm>

#include "mlib/utility/parser/xml.h"
#include "mlib/utility/string.h"

namespace mlib
{
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

    std::replace(
      properties_raw.begin(),
      properties_raw.end(),
      '\'',  '\"');

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

  XMLHandler::
  XMLHandler() {}

  XMLHandler::
  XMLHandler(std::string filename, bool is_file)
  {
    std::string text = "";

    if(is_file)
      {
        std::ifstream infile(filename);
        std::string line;
        while(std::getline(infile, line))
          {
            text += " " + line;
          }
      }
    else
      {
        text = filename;
      }

    while((0<text.find('<'))&&(text.find('<')<text.size()))
      {
        XMLEntry entry = process_XML_text(text);
        if(entry.label == "xml")
          {
            for(std::map<std::string, std::string>::iterator it = entry.properties.begin();
                it != entry.properties.end(); ++it)
              {
                std::string a = (*it).first;
                std::string b = (*it).second;
                xml.insert(std::make_pair<std::string, std::string>(a.c_str(), b.c_str()));
              }
          }
        else if(entry.label == "xml-stylesheet")
          {
            for(auto it = entry.properties.begin(); it != entry.properties.end(); ++it)
              {
                std::string a = (*it).first;
                std::string b = (*it).second;
                xml_stylesheet.insert(std::make_pair<std::string, std::string>(a.c_str(),
                                                                               b.c_str()));
              }
          }
        text = text.erase(entry.init, entry.end - entry.init);

        if(entry.label != "xml-stylesheet" && entry.label != "xml")
          {
            for(auto it = entry.properties.begin(); it != entry.properties.end(); ++it)
              {
                std::string a = (*it).first;
                std::string b = (*it).second;
                properties.insert(std::make_pair<std::string, std::string>(a.c_str(),
                                                                           b.c_str()));
              }
            xml_entries.insert(std::make_pair<std::string, XMLHandler>
                               (entry.label.c_str(), XMLHandler(entry.text,
                                                                false)));
          }
      }
    val_text = text;
  }

  void
  XMLHandler::
  print(int indent)
  {
    if(xml.size() > 0)
      {
        std::cout << "<?xml ";
        for(auto it = xml.begin(); it!= xml.end(); ++it)
          {
            std::cout << " " << it->first << "=\"" << it->second << "\" ";
          }
        std::cout << "?>" << std::endl;
      }

    if(xml_stylesheet.size() > 0)
      {
        std::cout << "<?xml_stylesheet ";
        for(auto it = xml_stylesheet.begin(); it!= xml_stylesheet.end(); ++it)
          {
            std::cout << " " << it->first << "=\"" << it->second << "\" ";
          }
        std::cout << "?>" << std::endl;
      }
    for(auto it = xml_entries.begin(); it!= xml_entries.end(); ++it)
      {
        for(int i = 0; i < indent; i++)
          {
            std::cout << " ";
          }
        std::cout << "<" << it->first;

        for(auto it = properties.begin(); it!= properties.end(); ++it)
          {
            std::cout << " " << it->first << "=\"" << it->second << "\" ";
          }
        std::cout << ">" << std::endl;
        for(int i = 0; i < indent; i++)
          {
            std::cout << " ";
          }
        std::cout << it->second.val_text<< std::endl;
        it->second.print(indent+1);
        for(int i = 0; i < indent; i++)
          {
            std::cout << " ";
          }
        std::cout << "<" << it->first  << ">" << std::endl;
      }

  }
  //
  // std::map<std::string, std::string>
  // get_property(std::string& text)
  // {
  //   std::map<std::string, std::string> result;
  //   std::replace(
  //     text.begin(),
  //     text.end(),
  //     '\"',  '\'');
  //
  //   std::replace(
  //     text.begin(),
  //     text.end(),
  //     '=',  ' ');
  //
  //   auto parts = split(text, "\' ");
  //   for(unsigned int i = 0; i<parts.size(); ++i)
  //     {
  //       std::vector<std::string>  name_and_val = split(parts[i], "\'");
  //       if(name_and_val[0]!="" && name_and_val.size()>1)
  //         result[remove_white_spaces(name_and_val[0])] = name_and_val[1];
  //     }
  //   return result;
  // }
  //
  // std::map<std::string, std::string>
  // process_meta_tag(const std::string& meta_tag,
  //                  std::string& text)
  // {
  //   std::string xml_stylesheet_property_raw = "";
  //   std::map<std::string, std::string> result;
  //   if(text.find(meta_tag) > 0)
  //     {
  //       std::string label = meta_tag;
  //       unsigned int init = text.find(meta_tag) + label.size() + 1;
  //       unsigned int end  = text.find("?>", init) - 1;
  //       xml_stylesheet_property_raw = text.substr(init, end - init);
  //       text.erase(text.find(meta_tag), text.find("?>",
  //                                                 init) - text.find(meta_tag) + 2);
  //     }
  //   return get_property(xml_stylesheet_property_raw);
  // }
  //
  // XMLEntry
  // get_entry(std::string& text)
  // {
  //   unsigned int pos      = text.find("<");
  //   unsigned int start    = pos;
  //   unsigned int with     = 0;
  //   unsigned int without  = 1;
  //
  //   while(with != without)
  //     {
  //       unsigned int next = text.find("<", pos+1);
  //       if(text.at(next+1) == '/')
  //         {
  //           with ++;
  //         }
  //       else
  //         {
  //           without ++;
  //         }
  //       pos = next;
  //     }
  //
  //   unsigned int close = pos;
  //   unsigned int end = text.find(">", close+1);
  //
  //   unsigned int label_init = text.find_first_not_of(" ", start) +1;
  //   unsigned int label_end =
  //     std::min(
  //       text.find(" ", label_init),
  //       text.find(">", label_init)
  //     );
  //   std::string label = text.substr(label_init, label_end - label_init);
  //
  //   std::string properties_raw = text.substr(label_end, text.find(">",
  //                                                                 label_end)-label_end);
  //   std::map<std::string, std::string> properties = get_property(properties_raw);
  //
  //   std::string txt = text.substr(
  //                       text.find(">", label_end) + 1,
  //                       text.find("<", label_end) - text.find(">", label_end) -1);
  //
  //   XMLEntry entry(label, txt);
  //   auto it = properties.begin();
  //   auto end_iterator = properties.end();
  //   for(; it!=end_iterator; ++it)
  //     entry.add_property(it->first, it->second);
  //
  //   std::string next_text =  text.substr(text.find("<", label_end),
  //                                        close - text.find("<", label_end));
  //   while(next_text.find("<")<next_text.size())
  //     {
  //       XMLEntry sub_entry = get_entry(next_text);
  //       entry.add(label, sub_entry);
  //   }
  //   text.erase(start, end-start +1);
  //
  //   return entry;
  // }
  //

  //
  // void
  // XMLHandler::
  // set_style(std::string style_)
  // {
  //   xml_stylesheet["href"] = style_;
  // }
  //
  // void
  // XMLHandler::
  // add(XMLEntry new_entry)
  // {
  //   xml_entries.push_back(std::make_shared<XMLEntry>(new_entry));
  // }
  //
  // void
  // XMLHandler::
  // print()
  // {
  //   std::cout << get_file() << std::endl;
  // }
  //
  // std::string
  // XMLHandler::
  // get_file(unsigned int depth)
  // {
  //   std::string text;
  //
  //   {
  //     text += "<?xml ";
  //     std::map<std::string, std::string>::iterator itl  = xml_header.begin();
  //     std::map<std::string, std::string>::iterator endl = xml_header.end();
  //     for(; itl!=endl; ++itl)
  //       {
  //         text +=  itl->first + "=\'" + itl->second + "\' ";
  //       }
  //     text += "?> \n";
  //   }
  //   if(xml_stylesheet["href"]!="")
  //     {
  //       text += "<?xml-stylesheet ";
  //       std::map<std::string, std::string>::iterator itl  = xml_stylesheet.begin();
  //       std::map<std::string, std::string>::iterator endl = xml_stylesheet.end();
  //       for(; itl!=endl; ++itl)
  //         {
  //           text +=  itl->first + "=\'" + itl->second + "\' ";
  //         }
  //       text += "?> \n";
  //     }
  //
  //   {
  //     std::map<std::string, std::shared_ptr<XMLEntry>>::iterator itl  =
  //                                                     xml_entries.begin();
  //     std::map<std::string, std::shared_ptr<XMLEntry>>::iterator endl =
  //                                                     xml_entries.end();
  //     for(; itl!=endl; ++itl)
  //       {
  //         text +=  itl->get()->append_string();
  //       }
  //   }
  //
  //   return text;
  // }
  //
  // void
  // XMLHandler::
  // save(const std::string& filename)
  // {
  //   std::ofstream outputFile(filename);
  //   outputFile << get_file();
  // }
  //
  // void
  // XMLHandler::
  // set_xml_header(std::string name, std::string val)
  // {
  //   xml_header[name] = val;
  // }
  //
  // void
  // XMLHandler::
  // set_xml_stylesheet(std::string name, std::string val)
  // {
  //   xml_stylesheet[name] = val;
  // }
  //
  // XMLEntry*
  // XMLHandler::
  // get_element(const unsigned int& i)
  // {
  //   return xml_entries[i].get();
  // }
  //
  XMLHandler
  XMLHandler::
  operator[](const std::string& s)
  {
    return xml_entries[s];
  }

  std::string
  XMLHandler::
  operator()()
  {
    return this->val_text;
  }
}
