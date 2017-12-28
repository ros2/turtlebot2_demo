/**
 * @file /ecl_mpl/src/test/enable_if.cpp
 *
 * @brief Unit tests for the enable_if template.
 *
 * @date July 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/mpl/enable_if.hpp"
#include "../../include/ecl/mpl/bool.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */
/*****************************************************************************
** Using
*****************************************************************************/

using ecl::enable_if;

/*****************************************************************************
** Classes
*****************************************************************************/

namespace ecl {
namespace mpl {
namespace tests {

/*****************************************************************************
** Is Float
*****************************************************************************/

template <typename T>
class is_float : public ecl::False {};

template <>
class is_float<float> : public ecl::True {};

template <>
class is_float<double> : public ecl::True {};

/**
 * @brief Default test class.
 */
template <typename T, typename Enable = void>
class TestDude {
public:
	bool isSpecialisation() { return false; }
};
/**
 * @brief Specialisation of the test class for floats only.
 */
template <typename T>
class TestDude< T, typename enable_if< is_float<T> >::type > {
public:
	bool isSpecialisation() { return true; }
};

} // namespace tests
} // namespace mpl
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::mpl::tests;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(EnableIfTests,oneshot) {
	TestDude<float> float_dude;
	TestDude<int> int_dude;
    bool result;
    result = float_dude.isSpecialisation();
    EXPECT_EQ(true, result);
    result = int_dude.isSpecialisation();
    EXPECT_EQ(false, result);
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
