/**
 * @file src/examples/random_number_generator.cpp
 *
 * @brief Demos the random number generator.
 *
 * @date July, 2014
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include "../../include/ecl/time/random_number_generator.hpp"

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main()
{

  ecl::RandomNumberGenerator<float> random_number_generator;
  int n = 5;

  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                   Uniform Numbers" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << std::endl;

  std::cout << "Default (0, 1):" << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.uniform();
  }
  std::cout << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.uniform();
  }
  std::cout << std::endl;
  std::cout << "Over Range (-5, 5): " << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.uniform(5.0);
  }
  std::cout << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.uniform(5.0);
  }
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                      Guassian" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << std::endl;

  std::cout << "std = 1, avg = 0: " << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(1.0);
  }
  std::cout << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(1.0);
  }
  std::cout << std::endl;

  std::cout << "std = 1, avg = 3: " << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(1.0, 3.0);
  }
  std::cout << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(1.0, 3.0);
  }
  std::cout << std::endl;

  std::cout << "std = 3, avg = 5: " << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(3.0, 5.0);
  }
  std::cout << std::endl;
  for (int i = 0; i < n; ++i)
  {
    std::cout << "  " << random_number_generator.gaussian(3.0, 5.0);
  }
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                      Passed" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << std::endl;
  return 0;
}
