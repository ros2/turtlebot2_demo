/**
 * @file /src/examples/constructors.cpp
 *
 * @brief Demos constructor usage on your platform.
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
** Classes
*****************************************************************************/

class A {
    public:
        A(int i) : var(i) { std::cout << "Constructor" << std::endl; }
        A(const A& a) { std::cout << "Copy constructor" << std::endl; }
        ~A() { std::cout << "Destructor" << std::endl; }
        void f() {};
    private:
        int var;
};

/*****************************************************************************
** Functions
*****************************************************************************/

A f() {
    return A(3);
}

A g() {
    A a(4);
    return a;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "            Normal Constructor : A a(3)" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    A a1(3); a1.f();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "             Copy Constructor : A a2(a)" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    A a2(a1); a2.f();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "       Constructor Assignment w/ Function : A a3 = f();" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    A a3 = f(); a3.f();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << " Constructor Assignment w/ Non Temp Function : A a4 = g();" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    A a4 = g(); a4.f();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  Program Finished" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;


    return 0;
}
/**
 * @endcond
 */
