/**
 * @file /include/ecl/errors/run_time_functions.hpp
 *
 * @brief Functions for error-checking and program termination.
 *
 * Functions for error-checking and program termination.
 *
 * @date April, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ERRORS_ERROR_FUNCTIONS_HPP_
#define ECL_ERRORS_ERROR_FUNCTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <ecl/config/macros.hpp>
#include "handlers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Functions [run_time_assert]
*****************************************************************************/

#if defined(NDEBUG) || defined(ECL_NDEBUG)
    #define ecl_run_time_assert(req,loc,msg) ((void)0)
#else
    /**
     * @brief Condition test, aborts with a programmable error string if it fails.
     *
     * Tests an assertion and if it fails, it writes a message (file, line# and
     * warning) to stderror before aborting the program (and if on *nix, core
     * dumping). This allows a customised user string for the error message.
     *
     * This function is only available if NDEBUG is not set (i.e. debug mode).
     *
     * @param requirement : logic expression to test for truthfulness.
     * @param location : code location, use the LOC macro.
     * @param msg : customised error string.
     *
     * @sa LOC
     **/
    ecl_errors_PUBLIC void ecl_run_time_assert ( bool requirement, const char* location, const char* msg);
    /**
     * @brief Condition test, aborts with a predefined error string if it fails.
     *
     * Tests an assertion and if it fails, it writes a message (file, line# and
     * warning) to stderror before aborting the program (and if on *nix, core
     * dumping). This uses a pre-defined error flag to generate the error message.
     *
     * This function is only available if NDEBUG is not set (i.e. debug mode).
     *
     * @param requirement : logic expression to test for truthfulness.
     * @param location : code location, use the LOC macro.
     * @param type : predefined error message tag.
     *
     * @sa LOC.
     **/
    ecl_errors_PUBLIC void ecl_run_time_assert ( bool requirement, const char* location, ecl::ErrorFlag type);
#endif

/*****************************************************************************
** Functions [abort]
*****************************************************************************/

/**
 * @brief Puts a customised error string to stderr before aborting the program.
 *
 * Writes an error message to stderror before promptly aborting. Note that this one
 * is not affected by the presence/absence of the macro NDEBUG.
 * @param location : code location, use the LOC macro.
 * @param msg : customised error string.
 *
 * @sa LOC
 **/
ecl_errors_PUBLIC void ecl_run_time_abort ( const char* location, const char* msg = "Abort procedure called.");
/**
 * @brief Puts a predefined error string to stderr before aborting the program.
 *
 * Writes an error message to stderror before promptly aborting with a pre-defined error message.
 * Note that this one is not affected by the presence/absence of the macro NDEBUG.
 * @param location : code location, use the LOC macro.
 * @param type : predefined error message type tag.
 *
 * @sa LOC
 **/
ecl_errors_PUBLIC void ecl_run_time_abort ( const char* location,  ecl::ErrorFlag type );

#endif /* ECL_ERRORS_ERROR_FUNCTIONS_HPP_*/
