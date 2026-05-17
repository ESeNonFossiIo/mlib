#include "../test.h"

#include "mlib/math/matrix/matrix.h"

#include <iostream>
#include <stdexcept>

using namespace mlib;

int main()
{
    print_title("Matrix - uncovered paths");

    // Matrix(rows, cols, T* elements) with non-null array
    double arr[] = {1.0, 2.0, 3.0, 4.0};
    Matrixd M(2, 2, arr);
    std::cout << " M[0] = " << M[0] << std::endl;

    // const operator[](i)
    const Matrixd& cM = M;
    std::cout << " cM[1] = " << cM[1] << std::endl;

    // element(i,j) const
    std::cout << " cM.element(0,1) = " << cM.element(0, 1) << std::endl;

    // element(i,j) non-const
    M.element(1, 0) = 9.0;
    std::cout << " M.element(1,0) = " << M.element(1, 0) << std::endl;

    // resize with non-null array
    double arr2[] = {7.0, 8.0, 9.0, 10.0};
    Matrixd N(2, 2);
    N.resize(2, 2, arr2);
    std::cout << " N[0] = " << N[0] << std::endl;

    // operator/(T a)
    Matrixd D = M / 2.0;
    std::cout << " D[0] = " << D[0] << std::endl;

    // operator*=(Matrix)
    Matrixd A({{1.0, 0.0}, {0.0, 1.0}});
    A *= M;
    std::cout << " A[0] = " << A[0] << std::endl;

    // operator~() — transpose via ~
    Matrixd T2 = ~M;
    std::cout << " T2[1] = " << T2[1] << std::endl;

    // range_check throw for row OOB
    try {
        double v = cM(5, 0);
        (void)v;
    } catch (const std::range_error&) {
        std::cout << " row_oob = 1" << std::endl;
    }

    // range_check throw for col OOB
    try {
        double v = cM(0, 5);
        (void)v;
    } catch (const std::range_error&) {
        std::cout << " col_oob = 1" << std::endl;
    }

    // degenerate Matrix(rows,cols,T*) throw
    try {
        Matrixd bad(0, 2, nullptr);
    } catch (const std::range_error&) {
        std::cout << " degen_ctor = 1" << std::endl;
    }

    // degenerate resize throw
    try {
        Matrixd mR(2, 2);
        mR.resize(0, 2);
    } catch (const std::range_error&) {
        std::cout << " degen_resize = 1" << std::endl;
    }

    return 0;
}
