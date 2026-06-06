#include "../test.h"

#include "numerix/coding/huffman.h"

using namespace numerix;

int main()
{
    print_title("Huffman coding");

    HuffmanCoding test("acabbaacbabc");

    test.print_counter();
}
