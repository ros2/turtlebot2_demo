/**
 * @file /src/lib/run_time_functions.cpp
 *
 * @brief Implementation for the error functions.
 *
 * @date April, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdio>
#include "../../include/ecl/errors/run_time_functions.hpp"
#include "../../include/ecl/errors/handlers.hpp"

/*****************************************************************************
** Function Implementation [assert]
*****************************************************************************/

#if defined(NDEBUG) || defined(ECL_NDEBUG)
#else
    void ecl_run_time_assert ( bool requirement, const char* location, const char* msg)
    {
        if ( !requirement )
        {
            fputs ("Location : ", stderr);
            fputs ( location ,stderr);
            fputs ("\n", stderr);
            fputs ("Message  : ", stderr);
            fputs ( msg, stderr);
            fputs ( "\n", stderr);
            ::abort();
        }
    }
    void ecl_run_time_assert ( bool requirement, const char* location, ecl::ErrorFlag type)
    {
        if ( !requirement )
        {
            fputs ("Location : ", stderr);
            fputs ( location ,stderr);
            fputs ("\n", stderr);
            fputs ("Message  : ", stderr);
            fputs ( ecl::Error(type).what(), stderr);
            fputs ( "\n", stderr);
            ::abort();
        }
    }
#endif


/*****************************************************************************
** Function Implementation [abort]
*****************************************************************************/

void ecl_run_time_abort ( const char* location, const char* msg)
{
    fputs ( "Location : ", stderr);
    fputs ( location, stderr);
    fputs ("\nMessage  : ", stderr);
    fputs ( msg, stderr);
    fputs ( "\n", stderr);
    ::abort();
}

void ecl_run_time_abort ( const char* location,  ecl::ErrorFlag type )
{
    fputs ( "Location : ", stderr);
    fputs ( location,stderr);
    fputs ("\nMessage  : ", stderr);
    fputs ( ecl::Error(type).what(), stderr);
    fputs ( "\n", stderr);
    ::abort();
}

