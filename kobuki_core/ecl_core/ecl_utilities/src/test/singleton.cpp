/**
 * @file /src/test/singleton.cpp
 *
 * @brief Unit Test for the @ref ecl::Singleton "Singleton" class.
 *
 * @date April 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/utilities/singleton.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
 * Classes
 ****************************************************************************/

namespace ecl {
namespace utilities {
namespace tests {

class TestSingleton : public ecl::Singleton<TestSingleton> {

	friend class ecl::Singleton<TestSingleton>;

    public:
        int value() { return data; }

    protected:
        TestSingleton() : data(32) {}

    private:
        int data;
};

}}}


/*****************************************************************************
** Convenience Macro
*****************************************************************************/

#define TestInstance ecl::utilities::tests::TestSingleton::instance()

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Singleton,instantiation) {
	int i = TestInstance.value();
    EXPECT_EQ(32,i);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}

/**
 * @endcond
 */
