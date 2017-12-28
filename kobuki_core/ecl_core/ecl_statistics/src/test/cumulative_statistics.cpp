/**
 * @file /src/test/covariance_ellipsoids.cpp
 *
 * @brief Unit Test for covariance ellipsoids.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/statistics/cumulative_statistics.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CumulativeStatistics, cumulative) {
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                 Cumulative Statistics" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;
    ecl::CumulativeStatistics<float> statistics;
    std::cout << "Size: " << statistics.size() << std::endl;
    std::cout << "  Mean: " << statistics.mean() << std::endl;
    std::cout << "  Variance: " << statistics.variance() << std::endl;
    EXPECT_FLOAT_EQ(0.0, statistics.size());
    EXPECT_FLOAT_EQ(0.0, statistics.mean());
    EXPECT_FLOAT_EQ(0.0, statistics.variance());
    statistics.push_back(1.0);
    std::cout << "Size: " << statistics.size() << std::endl;
    std::cout << "  Mean: " << statistics.mean() << std::endl;
    std::cout << "  Variance: " << statistics.variance() << std::endl;
    EXPECT_FLOAT_EQ(1.0, statistics.size());
    EXPECT_FLOAT_EQ(1.0, statistics.mean());
    EXPECT_FLOAT_EQ(0.0, statistics.variance());
    statistics.push_back(2.0);
    statistics.push_back(3.0);
    statistics.push_back(4.0);
    statistics.push_back(5.0);
    std::cout << "Size: " << statistics.size() << std::endl;
    std::cout << "  Mean: " << statistics.mean() << std::endl;
    std::cout << "  Variance: " << statistics.variance() << std::endl;
    EXPECT_FLOAT_EQ(5.0, statistics.size());
    EXPECT_FLOAT_EQ(3.0, statistics.mean());
    EXPECT_FLOAT_EQ(2.5, statistics.variance());
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


