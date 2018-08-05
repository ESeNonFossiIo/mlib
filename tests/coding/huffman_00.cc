#include "../test.h"

#include "mlib/coding/huffman.h"

using namespace mlib;

int main()
{
  print_title("Huffman coding");

  HuffmanCoding test("acabbaacbabc");

  test.print_counter();
}
