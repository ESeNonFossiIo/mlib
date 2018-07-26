#include "mlib/math/matrix/matrix.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Matrix" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  Matrix<double> m(2,2);
  m(0,0) = 10;
  m(1,0) = 11;

  Matrix<double> n(2,2);
  n(0,0) = 10;
  n(1,0) = 11;

  for(unsigned int i =0; i<2; i++)
    for(unsigned int j =0; j<2; j++)
      std::cout << (n+m)(i,j) << std::endl;

  std::cout << m *n;

  std::vector<std::vector<double>> v = {{1,0,1},{0,1,0},{0,0,1}};
  Matrix<double> vv(v);
  std::cout << vv;

  std::vector<std::vector<double>> p1 = {{1},{0},{0}};
  std::vector<std::vector<double>> p2 = {{0},{1},{0}};
  std::vector<std::vector<double>> p3 = {{0},{0},{1}};
  Matrix<double> e1(p1);
  Matrix<double> e2(p2);
  Matrix<double> e3(p3);

  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << vv(0,2) << std::endl;
  std::cout << vv(2,0) << std::endl;
  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << vv *e1 << std::endl;
  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << vv *e2 << std::endl;
  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << vv *e3 << std::endl;
  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << vv;
  std::cout <<
            "*************************************************" <<
            std::endl;
  std::cout << e1.transpose() * vv << std::endl;
  std::cout <<
            "*************************************************" <<
            std::endl;

}
