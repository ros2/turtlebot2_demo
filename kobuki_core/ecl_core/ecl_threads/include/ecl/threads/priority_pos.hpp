/**
 * @file /include/ecl/threads/priority_pos.hpp
 *
 * @brief Posix priority scheduling.
 *
 * @date May 2009.
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_PRIORITY_POS_HPP_
#define ECL_THREADS_PRIORITY_POS_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <unistd.h>
#include <string>
#include <sstream>
#include <errno.h>
#include <sys/resource.h>
#include <ecl/config/ecl.hpp>
#include <ecl/exceptions/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "priority_common.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Sets the priority to the specified level.
 *
 * Posix scheduling has several policies (cf. windows which has only one),
 * we only use two.
 *
 * - SCHED_OTHER : the traditional priority group.
 * - SCHED_RR : a real time priority group.
 *
 * The traditional policy group uses 'niceness' to weight process scheduling
 * with a standard automatic scheduler as seen on most systems. The ecl
 * Priority levels that are spread across this weighting system include
 * (from lowest priority to highest) :
 *
 * - BackgroundPriority
 * - LowPriority
 * - NormalPriority
 * - HighPriority
 * - CriticalPriority
 *
 * The real time priority group elevates processes above the traditional
 * group, so the scheduler will always allow the realtime priority process
 * to work before a traditional priority process. These too are given
 * priority levels (usually 1-99), so that higher level priorities
 * will similarly dominate lower level priorities. Threads with the same
 * realtime priority will be put into a round-robin. The ecl Priority
 * levels that are spread across the realtime group include (from lowest to
 * highest priority):
 *
 * Be careful with the realtime priorities as it will cause the current
 * process to dominate processes with lesser priorities (i.e. potentially locking
 * up the machine or starving other threads/processes).
 *
 * - RealTimePriority1
 * - RealTimePriority2
 * - RealTimePriority3
 * - RealTimePriority4
 *
 * One further note, higher level priorities cannot be enabled by a normal
 * user on a linux system (usually anything higher than NormalPriority). You have
 * several options in this case:
 * - Run the binary as root.
 * - Unlock the SYS_CAP_NICE posix file capability on the binary (not possible yet on linux systems).
 * - Edit the user's limits in <i>/etc/security/limits.conf</i>.
 *
 * An example of editing this file for a user <i>snorri</i> to enable both realtime priorities
 * to level 95 and the maximum niceness setting on a typical linux box:
 * @code
 * snorri          -       nice            -20
 * snorri          -       rtprio           95
 * @endcode
 *
 * You can observe the process' prioritisation via the ps command. A useful set of flags
 * that lets you determine rt priority/niceness and a few other vitals is (see man ps for more
 * details):
 *
 * @code
 * ps -C <process_name> -m -o pid,rtprio,ni,pri,vsize,rssize,pmem,pcpu,comm
 * @endcode
 *
 * @param priority_level : the priority level requested for this process.
 * @return bool : true if success, false if failed and exceptions aren't enabled.
 * @exception StandardException : throws if configuration fails.
 */
bool ECL_PUBLIC set_priority(Priority priority_level) ecl_debug_throw_decl(StandardException);
/**
 * @brief Returns the process' current priority level.
 *
 * @return Priority : the process priority enumeration (UnknownPriority if it fails and exceptions aren't enabled).
 * @exception StandardException : if exceptions enabled, throws if the call fails.
 */
Priority ECL_PUBLIC get_priority() ecl_debug_throw_decl(StandardException);

/**
 * @brief Print priority diagnostics to a string.
 * @return string : insert into a stream to display/log.
 * @exception StandardException : if exceptions enabled, throws if any call fails.
 */
std::string ECL_PUBLIC print_priority_diagnostics() ecl_debug_throw_decl(StandardException);

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace threads {

/**
 * @brief Worker function to configure the real time priorities.
 *
 * Configures the process with real time scheduling and prioritisation.
 *
 * Warning: the specified level will be ignored if you are using PAM and
 * it is larger than the user's specified rtprio limit. In this case, it
 * defaults back to its parent's priority.
 *
 * @param policy : either SCHED_OTHER or SCHED_RR.
 * @param priority_level : a value converted from one of the PriorityLevel abstractions.
 * @return bool : true if it works, false if not (or unsupported).
 *
 * @exception StandardException : if exceptions are enabled, throws if the operation fails or is not supported.
 */
bool ECL_LOCAL set_real_time_priority(int policy,int priority_level) ecl_debug_throw_decl(StandardException);

} // namespace threads
} // namespace ecl

/*****************************************************************************
** Interface [Exceptions]
*****************************************************************************/

#if defined(ECL_HAS_EXCEPTIONS)
namespace ecl {
namespace threads {

/*****************************************************************************
** Interface [Priority Exceptions]
*****************************************************************************/
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by priority function calls.
 *
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 */
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>usleep and nanosleep</i> posix
 * functions used in the sleeper classes.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 */
inline StandardException ECL_LOCAL throwPriorityException(const char* loc ) {
	int error_result = errno;
    switch (error_result) {
        case ( EINVAL ) : return StandardException(loc, ecl::InvalidInputError, "The specified param structure or priority group was invalid.");
        case ( ESRCH  ) : return StandardException(loc, ecl::InvalidInputError, "The process specified could not be found.");
        case ( EPERM  ) : return StandardException(loc, ecl::PermissionsError, "The caller does not have the appropriate privileges for realtime scheduling (http://snorriheim.dnsdojo.com/doku/doku.php/en:linux:admin:priorities).");
        case ( EACCES ) : return StandardException(loc, ecl::PermissionsError, "The caller does not have the appropriate privileges for elevating the process priority by reducing the niceness value (http://snorriheim.dnsdojo.com/doku/doku.php/en:linux:admin:priorities).");
		default         :
		{
			std::ostringstream ostream;
			ostream << "Unknown posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}


}; // namespace threads
} // namespace ecl
#endif /* ECL_HAS_EXCEPTIONS */
#endif /* ECL_IS_POSIX */
#endif /* ECL_THREADS_PRIORITY_POS_HPP_ */


//        void yield();
//        void capabilities() throw(StandardException);


///*
// * Yields this process (must be a real-time process) to lower priority processes.
// * In effect, this places the process at the back of the cpu queue, even if there
// * are lower priority processes ahead of it.
// */
//inline void Process::yield()
//{
//    sched_yield();
//}

///*
// * Print out the posix capabilities of your program.
// */
//inline void Process::capabilities() throw(StandardException)
//{
//    cap_t cap = cap_get_proc();
//
//    if (!cap) {
//        throw StandardException(LOC,"Could not retrieve posix characteristics.","Was not possible to retrieve the processes' posix capability list.");
//    }
//    std::cout << "Posix capabilities " << cap_to_text(cap,NULL) << std::endl;
//    cap_free(cap);
//}

