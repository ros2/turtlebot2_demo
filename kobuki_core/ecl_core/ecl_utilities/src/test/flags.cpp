/**
 * @file /src/test/flags.cpp
 *
 * @brief Unit Test for the @ref ecl::Flags "Flags" class.
 *
 * @date February 2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/utilities/flags.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */

#define FLAGS_DEBUG

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace utilities {
namespace tests {

/*****************************************************************************
** Enum
*****************************************************************************/

enum Setting
{
    Nada = 0x00, // the empty flag.
    Fast = 0x01, // Group speed
    Slow = 0x02,
    Bright = 0x10, // Group colour
    Dark = 0x20,
    Red = 0x40,
    Gold = 0x80,
    FastGold = 0x81, // Combinations (convenient setting and checking (w/ testFlag()))
};

/*****************************************************************************
** Helper Piper
*****************************************************************************/
/**
 * The helper function that makes it convenient for piping enums into a flags variable.
 **/
Flags<Setting> operator|(Setting flagOne, Setting flagTwo) { return Flags<Setting>(flagOne) | flagTwo; }

enum Property
{
    Fixed = 0x01,
    Dynamic = 0x02
};

/**
 * Useful to define your enum with the flag name and a pluralised typedef like this for
 * ease of notation.
 */
typedef Flags<Setting> Settings;

/*****************************************************************************
** Illegal combination handler
*****************************************************************************/
/**
 * Restricting combinations via an accessor.
 */
class A
{
public:
    void configure(Settings s)
    {
        if ( ( s.testFlag(Gold) && s_.testFlag(Dark) ) ||
             ( s_.testFlag(Gold) && s.testFlag(Dark) ) ||
             ( s.testFlag(Gold) && s.testFlag(Dark) ) )
        {
            #ifdef FLAGS_DEBUG
              std::cout << "This combination is not permitted." << std::endl;
            #endif
            return;
        } else {
            s_ = s;
        }
    }
    Settings settings() { return s_; }

private:
    Settings s_;
};

}}} // namespaces

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::utilities::tests;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Flags, initialisation) {

  Settings settings;
  settings = Fast | Red;
  // settings = Fixed; // <-- Illegal call, incompatible enum used.
#ifdef FLAGS_DEBUG
  std::cout << "Fast & Red: " << settings << std::endl;
#endif
  EXPECT_EQ(Fast|Red,settings);
}

TEST(Flags, Operators) {
  Settings settings;
  settings = Fast | Red;
  settings &= Fast; // <-- Mask out everything but Fast
#ifdef FLAGS_DEBUG
  std::cout << "Fast: " << settings << std::endl;
#endif

  settings |= Gold; // <-- Add gold
#ifdef FLAGS_DEBUG
  std::cout << "Fast & Gold: " << settings << std::endl;
#endif
  EXPECT_EQ(FastGold,settings);
}

TEST(Flags, tests) {
  Settings settings;
  settings = Fast | Gold;
  EXPECT_EQ(settings.testFlag(Gold),true);
  EXPECT_EQ(settings.testFlag(Red),false);
  EXPECT_EQ(settings.testFlag(FastGold),true);
  settings.clear();
  EXPECT_EQ(settings.testFlag(Nada),true);

#ifdef FLAGS_DEBUG
  if ( settings.testFlag(Gold) ) {
      std::cout << "Gold Set" << std::endl;
  } else {
      std::cout << "Gold Not Set" << std::endl;
  }
  if ( settings.testFlag(Red) ) {
      std::cout << "Red Set" << std::endl;
  } else {
      std::cout << "Red Not Set" << std::endl;
  }
  if ( settings.testFlag(FastGold) ) {
      std::cout << "Fast & Gold Set" << std::endl;
  } else {
      std::cout << "Fast & Gold Not Set" << std::endl;
  }
#endif
}
TEST(Flags, combinations) {
    A a;
    a.configure(Gold|Dark); // fails to configure
    EXPECT_EQ(0,a.settings());
    a.configure(Gold);
    EXPECT_EQ(Gold,a.settings());
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
