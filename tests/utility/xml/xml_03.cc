#include "../../test.h"

#include "mlib/utility/parser/xml_old.h"
#include <iostream>

using namespace _mlib;

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

  XMLHandler* xml_handler;
  {
    xml_handler = new XMLHandler();
    xml_handler->set_xml_header("version",  "1.0");
    xml_handler->set_xml_header("encoding", "ISO-8859-1");
    xml_handler->set_xml_header("type", "text/xsl");
    xml_handler->set_style("image_metadata_stylesheet.xsl");

    XMLEntry dataset("dataset");

    XMLEntry name("name");
    XMLEntry comment("comment");
    XMLEntry images("images");

    dataset.add(name);
    dataset.add(comment);
    dataset.add(images);


    xml_handler->add(dataset);

  }
  XMLEntry image("image");
  image.add_property("file", "file_path");

  xml_handler->get_element(0)->get_element(2)->add(image);
  xml_handler->get_element(0)->get_element(2)->add(image);
  xml_handler->get_element(0)->get_element(2)->add(image);
  xml_handler->print();
  std::cout << "====================" << std::endl;
  return 0;
}
