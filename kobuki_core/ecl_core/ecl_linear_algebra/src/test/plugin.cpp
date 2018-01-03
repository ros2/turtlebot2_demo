/**
 * @file /ecl_linear_algebra/src/test.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 28/07/2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/linear_algebra.hpp"
#include <gtest/gtest.h>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::linear_algebra::MatrixXd;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(PluginTest,autoSucceedNeedsFix) {
    SUCCEED();
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	MatrixXd matrix(2,2);
	matrix << 1.0, 2.0, 3.0, 4.0;
	std::cout << matrix << std::endl;
	matrix.conservativeResize(4,4);
	std::cout << "After resizing.........." << std::endl;
	std::cout << matrix << std::endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
