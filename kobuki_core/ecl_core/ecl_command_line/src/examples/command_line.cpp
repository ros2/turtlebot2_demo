/**
 * @file /src/examples/command_line.cpp
 *
 * @brief Demo program for the tclap header library.
 *
 * @date May, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include "../../include/ecl/command_line.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using ecl::CmdLine;
using ecl::SwitchArg;
using ecl::ValueArg;
using ecl::UnlabeledValueArg;
using ecl::ArgException;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

    int test(0);
    bool debug(false);
    string nolabel;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                Command Line Parsing" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
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

        // (one word argument name, long description, req/not req, default value, type)
        UnlabeledValueArg<string> nolabelArg( "echo", "An unlabeled argument", false, "dude", "string"  );
        cmd.add(nolabelArg);

        /*********************
        ** Parse
        **********************/
        cmd.parse(argc,argv);
        debug = debugSwitch.getValue();
        test = intArg.getValue();
        nolabel = nolabelArg.getValue();

    } catch ( ArgException &e )
    {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
    }

    cout << "Test Integer: " << test << endl;
    if ( debug ) {
        cout << "Debug Switch (bool): true" << endl;
    } else {
        cout << "Debug Switch (bool): false" << endl;
    }
    std::cout << "Echoing unlabelled arg: " << nolabel << std::endl;
    std::cout << std::endl;

    return 0;
}



