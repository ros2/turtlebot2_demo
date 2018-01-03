/**
 * @file /src/utils/process_statistics.cpp
 *
 * @brief Detects parameter configurations for processes' on your system.
 *
 * This detects the availability and ranges of various process methods on
 * your particular system.
 *
 * - Niceness
 * - Real time priority range
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/threads/priority.hpp>

/*****************************************************************************
** Main
*****************************************************************************/

int main() {

    std::cout << ecl::print_priority_diagnostics() << std::endl;

    std::cout << "This is very much a work in progress...." << std::endl;

    return 0;
}


