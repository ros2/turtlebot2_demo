/**
 * @file /ecl_linear_algebra/src/test/formatters.cpp
 *
 * @brief Unit test the matrix formatters.
 *
 * Can't easily do gtests on this as its just streaming to cout.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/linear_algebra.hpp"
#include <ecl/formatters.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Format;
using ecl::linear_algebra::Matrix2f;
using ecl::linear_algebra::Matrix2d;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::MatrixFormatter;
using ecl::linear_algebra::FloatMatrixFormatter;

/*****************************************************************************
** Main program
*****************************************************************************/

TEST(FormattersTest,autoSucceedNeedsFix) {
    SUCCEED();
}

int main(int argc, char **argv) {

    Vector2d v; v << 1.0, 2.0;
    Matrix2d m; m << 1.0, 2.0, 3.0, 4.134;
    Matrix2f mf; mf << 1.0, 2.0, 3.0, 4.134;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  MatrixFormatter" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    FloatMatrixFormatter<Vector2d> vector_format;
    std::cout << vector_format(v) << std::endl;
    FloatMatrixFormatter<Matrix2d> matrix_format;
    std::cout << matrix_format(m) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "              MatrixBase<Derived>::Formatter" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Vector2d::Formatter vector_format2(3);
    std::cout << vector_format2(v) << std::endl;
    Matrix2d::Formatter matrix_format2;
    std::cout << matrix_format2(m) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "              Format<MatrixType>" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Format<Vector2d> vector_format3;
    std::cout << vector_format3(v) << std::endl;
    Format<Matrix2f> matrix_format3; // Sneaking a test in for floats here
    std::cout << matrix_format3(mf) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "              Temporary/Prm Settings" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    matrix_format.precision(2);
    matrix_format.width(-1);
    std::cout << "Using perm settings (2,-1):" << std::endl;
    std::cout << matrix_format(m) << std::endl;
    std::cout << "Using temp settings (3, 6):" << std::endl;
    std::cout << matrix_format(m,3,6) << std::endl;
    std::cout << "Using perm settings (2,-1):" << std::endl;
    std::cout << matrix_format(m) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();

}
