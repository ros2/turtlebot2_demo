/**
 * @file /src/test/realpath.cpp
 *
 * @brief Unit Test for the realpath function.
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/filesystem/config.hpp>
#if defined(ECL_PRIVATE_HAS_POSIX_REALPATH)

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include "../../include/ecl/filesystem/realpath.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FilesystemTests,realpath) {
  std::string abs_path;
  std::string rel_path = "./experiment.dslam";

  ecl::realpath(rel_path, abs_path);
  // TODO : right now, we have no idea of where this will run. Once we have
  // a `pwd` like function later, insert it here to derive the test properly.
  //EXPECT_EQ(std::string("Foo"), abs_path);
  std::cout << "Absolute path: " << abs_path << std::endl;
  SUCCEED();
}

#endif /* ECL_PRIVATE_HAS_POSIX_REALPATH */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


