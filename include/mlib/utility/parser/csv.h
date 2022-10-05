#ifndef _MLIB_CSV_MANAGER_FILE_H__
#define _MLIB_CSV_MANAGER_FILE_H__

#include <sstream>      // std::stringstream
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <fstream>
#include <cassert>


#include "mlib/utility/string.h"

/** \addtogroup utility
 *  @{
 */
namespace mlib
{
  class CSVHandler
  {

  public:
    CSVHandler();

    CSVHandler(const std::string& filename,
               const std::string& sep_char=";",
               const bool title=true)
      :
      title(title)
    {
      std::string line;
      std::ifstream infile(filename);
      assert(infile.is_open());

      if(title)
        {
          std::getline(infile, line);
          labels = split(line,sep_char,true);
        }
      while(std::getline(infile, line))
        {
          std::vector<std::string> vals = split(line,sep_char,true);
          data.push_back(vals);
        }
    }

    void
    save(const std::string& configuration_filename)
    {
      std::ofstream outputFile(configuration_filename);

      if(title)
        {
          outputFile << labels[0];
          for(unsigned int j = 1; j<labels.size(); ++j)
            {
              outputFile << "\t;\t" << labels[j] ;
            }
          outputFile << std::endl;
        }

      for(unsigned int i = 0; i<data.size(); ++i)
        {
          outputFile << data[i][0];
          for(unsigned int j = 1; j<data[0].size(); ++j)
            {
              outputFile << "\t;\t" << data[i][j] ;
            }
          outputFile << std::endl;
        }
    }

    void
    add(const std::vector<std::string>& datum)
    {
      assert(datum.size() == labels.size());
      data.push_back(datum);
    }

    std::vector<std::string>
    get(const std::string& label)
    {
      int i = -1;

      for(unsigned int j = 0; j<labels.size(); ++j)
        if(label == labels[j]) i = j;

      assert(i>=0);
      std::vector<std::string> result;
      for(unsigned int j = 0; j<data.size(); ++j)
        result.push_back(data[j][i]);

      return result;
    }

    std::vector<std::string>
    get_labels()
    {
      return labels;
    }

  private:
    std::vector<std::string> labels;
    std::vector<std::vector<std::string>> data;
    bool title;
  };

}

/** @}*/
#endif //_MLIB_CSV_MANAGER_FILE_H__
