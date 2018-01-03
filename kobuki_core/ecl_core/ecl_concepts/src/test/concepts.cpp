/**
 * @file /src/test/concepts.cpp
 *
 * @brief Unit Test for concept checking.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include "../../include/ecl/concepts/blueprints.hpp"
#include "../../include/ecl/concepts/containers.hpp"
#include "../../include/ecl/concepts/macros.hpp"
#include "../../include/ecl/concepts/streams.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
** Classes
*****************************************************************************/

namespace ecl {
namespace concepts {
namespace tests {

template <typename T>
class Concept {
    public:
		ecl_compile_time_concept_test(Concept)
        {
            T t;
            t.apply();  // This is the characteristic function required by this concept.
        }
};

class A {
    public:
        void apply() {}; // Comment this line to cause the compile to fail.
};

class BluePrintCandidate {
public:
	typedef int base_type;
	base_type instantiate();
	void apply(base_type dude);
};

} // namespace tests
} // namespace concepts
} // namespace ecl

/*****************************************************************************
** Tests
*****************************************************************************/

using namespace ecl::concepts::tests;

// By default, the concept tests will pass. To cause it to fail
// comment the concept test functions and rebuild.


TEST(CompileTimeTests,macro) {
	ecl_compile_time_concept_check(Concept<A>);
	EXPECT_TRUE(true); // Should only fall over if compile time check is triggered.
}

TEST(CompileTimeTests,stream) {
	ecl_compile_time_concept_check(ecl::StreamConcept<std::ostream>);
	EXPECT_TRUE(true); // Should only fall over if compile time check is triggered.
}

TEST(CompileTimeTests,blueprint) {
	ecl_compile_time_concept_check(ecl::BluePrintConcept<BluePrintCandidate>);
	EXPECT_TRUE(true); // Should only fall over if compile time check is triggered.
}

TEST(CompileTimeTests,container) {
	ecl_compile_time_concept_check(ecl::DynamicContainerConcept< std::vector<int> >);
	EXPECT_TRUE(true); // Should only fall over if compile time check is triggered.
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

/**
 * @endcond
 */

