/**
 * @file /include/ecl/time/random_number_generator.hpp
 *
 * @brief Seeding and generating various random number generator distributions.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_RANDOM_NUMBER_GENERATOR_HPP_
#define ECL_TIME_RANDOM_NUMBER_GENERATOR_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <ecl/config.hpp>

#if defined(ECL_IS_WIN32)
  getpid = _getpid
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Random Number Generator
*****************************************************************************/
/**
 * @brief A random number generator
 */
template<typename T>
class RandomNumberGenerator
{
public:
  RandomNumberGenerator( const unsigned int randSeed )
  {
    srand( randSeed );
  }

  RandomNumberGenerator()
  {
    RandomNumberGenerator::seed();
  }

  T uniform( const T & range)
  {
    return (uniform()-0.5)*2.0*range;
  }

  T uniform()
  {
    return static_cast<T>(rand())/static_cast<T>(RAND_MAX);
  }

  T gaussian( const T & std, const T & mu = 0 )
  {
    T x1, x2, w, y1;
    static T y2;
    static bool use_last = false;

    if (use_last)  /* use value from previous call */
    {
      y1 = y2;
      use_last = false;
    }
    else
    {
      do {
          x1 = 2.0 * uniform() - 1.0;
          x2 = 2.0 * uniform() - 1.0;
          w = x1 * x1 + x2 * x2;
      } while ( w >= 1.0 );

      w = std::sqrt( (-2.0 * std::log( w ) ) / w );
      y1 = x1 * w;
      y2 = x2 * w;
      use_last = true;
    }

    return mu + y1 * std;
  }

private:
  /**
   * @brief Seed the system random number generator.
   *
   * Here we create a seed by hashing from three independent sources.
   * This is a more robust way to get a reliable seed that allows
   * you to get the typically para-optimal traits
   *
   * 1) fine grained (e.g. you want to call srand two times in a *very* short interval)
   * 2) low chance of collisions over a long period
   *
   * It also avoids having to use esoteric linux functions lick clock_gettime that
   * aren't available on all platforms.
   *
   * References:
   *
   * -h ttp://stackoverflow.com/questions/322938/recommended-way-to-initialize-srand
   * - http://burtleburtle.net/bob/hash/doobs.html
   */
  static void seed()
  {
    unsigned long a = clock();
    unsigned long b = ::time(NULL);
    unsigned long c = getpid();
    a=a-b;  a=a-c;  a=a^(c >> 13);
    b=b-c;  b=b-a;  b=b^(a << 8);
    c=c-a;  c=c-b;  c=c^(b >> 13);
    a=a-b;  a=a-c;  a=a^(c >> 12);
    b=b-c;  b=b-a;  b=b^(a << 16);
    c=c-a;  c=c-b;  c=c^(b >> 5);
    a=a-b;  a=a-c;  a=a^(c >> 3);
    b=b-c;  b=b-a;  b=b^(a << 10);
    c=c-a;  c=c-b;  c=c^(b >> 15);
    srand(c);
  }
};

} // namespace ecl


#endif /* ECL_TIME_RANDOM_NUMBER_GENERATOR_HPP_ */
