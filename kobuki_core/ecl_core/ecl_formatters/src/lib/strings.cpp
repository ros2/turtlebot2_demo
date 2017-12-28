/**
 * @file /src/lib/strings.cpp
 *
 * @brief Implementation for the string formatter.
 *
 * @date May, 2009.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/formatters/strings.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;

/*****************************************************************************
** Implementation [Format<string>]
*****************************************************************************/

Format<std::string>& Format<std::string>::operator () (int w, Alignment a)
{
    width(w);
    align(a);
    return *this;
}

Format<std::string>& Format<std::string>::operator()(const std::string &input_string)
{
    s =  input_string;
    ready_to_format = true;
    return *this;
}

/*****************************************************************************
* Implementation [temporary formatting]
*****************************************************************************/
Format<std::string>& Format<std::string>::operator() (const std::string &input_string, int w)
{
    width(w);
    s = input_string;
    ready_to_format = true;
    return *this;
}
Format<std::string>& Format<std::string>::operator() (const std::string &input_string, int w, Alignment a)
{
    width(w);
    align(a);
    s = input_string;
    ready_to_format = true;
    return *this;
}

}; // namespace ecl



