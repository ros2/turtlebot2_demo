/**
 * @file /src/test/command_line.cpp
 *
 * @brief Unit Test for the tclap header library.
 *
 * Unit Test for the tclap header library (ecl/command_line.hpp).
 *
 * @date May, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/command_line.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::CmdLine;
using ecl::SwitchArg;
using ecl::ValueArg;
using ecl::ArgException;

/*****************************************************************************
** Globals
*****************************************************************************/

static int myargc;
static char **myargv;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CommandLineTests,verify) {
//    try {
        // Supply a program description, argument separator (optional) and version number (optional).
        CmdLine cmd("This is a test program to test the command line parsing facilities provided by TCLAP.");
//        CmdLine cmd("This is a test program to test the command line parsing facilities provided by TCLAP.", ' ', "0.01");

        // Add a boolean (flag, name, description), sets the default state to false unless it is found on the command line.
        SwitchArg debugSwitch("d","debug","Enable debugging.");
        // Add a boolean (flag, name, description, default)
        // SwitchArg debug("d","debug","Enable debugging.", false);
        cmd.add(debugSwitch);

        // Add an int (flag,name,description,compulsory flag?,default value,help description of the type")
        ValueArg<int> intArg("i","integer","An integer argument for testing.",false,5,"integer");
        cmd.add(intArg);

        /*********************
        ** Parse
        **********************/
        cmd.parse(myargc,myargv); // RoS gtest 'make test' breaks here.
        bool debug = debugSwitch.getValue();
        int test = intArg.getValue();

        EXPECT_EQ(5,test);
        EXPECT_FALSE(debug);

    } catch ( ArgException &e ) {
    	ADD_FAILURE() << "Failed to parse commadn line arguments";
    }
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	myargc = argc;
	myargv = argv;

	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}


