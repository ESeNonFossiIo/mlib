#ifndef __MYLIB_XML_MANAGER_FILE_H__
#define __MYLIB_XML_MANAGER_FILE_H__

#include <sstream>      // std::stringstream
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <fstream>
#include <memory>
#include <mlib/utility/string.h>
/** \addtogroup utility
 *  @{
 */
namespace _mlib
{
  class XMLParser
  {
  public:
    XMLParser():
      is_header(false),
      is_leaf(true)
    {};

    void process_file(const std::string& filename)
    {
      std::ifstream infile(filename);
      std::string line;
      std::string text = "";
      while(std::getline(infile, line))
        {
          text += line;
        }
      process_txt(text);
    };

    void process_txt(std::string& text)
    {
      size_t first_tag = text.find('<');
      size_t offset = 0;
      if(text.at(first_tag+1)=='?')
        {
          offset    = 1;
          is_header = true;
        }

      size_t end_ft_1 = text.find(' ', first_tag);
      size_t end_ft_2 = text.find('>', first_tag);
      size_t end      = std::min(end_ft_1, end_ft_2);

      tag = text.substr(first_tag + 1 + offset, end - first_tag - 1 - offset);

      std::vector<std::string> props
      = split(
      text.substr(end_ft_1, end_ft_2 - end_ft_1),
      " ", false, false);
      for(auto it = props.begin();
          it != props.end(); ++it)
        {
          auto v = split(*it, "=", false, false);
          properties.insert(std::pair<std::string, std::string>(v[0], v[1]));
        }

      text = text.erase(first_tag, end_ft_2 + 1);

      while(first_tag != -1)
        {
          size_t init = text.find('>', first_tag + 1);
          size_t end;

          size_t lo;

          init = first_tag;
          // sol = extraxt from lo to fo
          // std::cout << ">>>>" <<  text.at(fo+1) << "<<<<\n";
          if(text.at(first_tag+1)=='?')
            {
              end = first_tag + 1;
            }
          else
            {
              size_t lo1 = text.find(' ', init);
              size_t lo2 = text.find('>', init);
              // std::cout << "-->>"<< lo1 << "//" << lo2 << "//" << lo << "<<<<\n";
              end = std::min(lo1, lo2);
              // std::cout << "-->>"<< lo1 << "//" << lo2 << "//" << lo << "<<<<\n";

              // std::cout << "###" << tag << "###" << std::endl;
              int status = 1;
              do
                {
                  size_t end1 = text.find("</"+tag, end+2);
                  size_t end2 = text.find("<"+tag,  end+2);
                  end = std::min(end1, end2);
                  if(end == end1)
                    {
                      status -= 1;
                    }
                  else
                    {
                      status += 1;
                    }
                  // std::cout << status << " - - " << end << " - " << text.substr(init + 1, end - init - 1)<< "\n";
                }
              while(status != 0);

              end += tag.size() + 3;
            }

          std::string sub_text = text.substr(init, end - init);
          std::cout << "-->>||" << sub_text << "||" << std::endl;

          text = text.erase(first_tag, end + 1);
          first_tag = text.find('<');
        };
    };

    void print_init(int n_spaces = 0)
    {
      std::string spaces = "";
      for(int i = 0; i < n_spaces; i++)
        spaces += " ";
      std::cout << spaces << "<" << tag;
      if(properties.size() > 0)
        {
          for(auto it = properties.begin();
              it != properties.end(); ++it)
            {
              std::cout << " " << it->first
                        << "=" << it->second
                        << "";
            }
        }
      std::cout << spaces << ">" << std::endl;
      std::cout << spaces << " " << val << std::endl;
    };

    void print_end(int n_spaces = 0)
    {
      std::string spaces = "";
      for(int i = 0; i < n_spaces; i++)
        spaces += " ";
      std::cout << spaces << "</" << tag << ">" << std::endl;
    };

    void print(int n_spaces = 0)
    {
      print_init(n_spaces);
      for(auto it = entry.begin();
          it != entry.end(); ++it)
        {
          it->print(n_spaces + 1);
        }
      print_end(n_spaces);
    };

  private:
    bool is_header;
    bool is_leaf;
    std::string tag;
    std::string val;
    std::map<std::string, std::string> properties;
    std::vector<XMLParser > entry;
  };

}

/** @}*/
#endif //__MYLIB_XML_MANAGER_FILE_H__
