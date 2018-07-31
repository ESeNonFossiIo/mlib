#include "../../test.h"

#include "mlib/utility/parser/xml.h"
#include <iostream>

using namespace mlib;

void test(std::string&& str)
{
  std::cout << "=======================" << std::endl;
  XMLEntry res = process_XML_text(str);
  std::cout << "label    : " << res.label << std::endl;
  std::cout << "text     : " << res.text << std::endl;
  for(auto it = res.properties.begin(); it != res.properties.end(); ++it)
    {
      std::cout << "property : " << it->first << " -> " << it->second <<
                std::endl;
    }
}

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - XMLHandler" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  {
    test("<text prop=\"prop1\"> txt </text>");
    test("<text prop=\"prop1\"> <text prop=\"prop1\"> txt </text> </text>");
    test("<?xml version=\"1.0\" ?><?xml-stylesheet href=\"prova.css\" type=\"text/xsl\" ?> ");
    test("<?xml-stylesheet href=\"prova.css\" type=\"text/xsl\"  test=\"\" ?>");
  }

  return 0;
}
