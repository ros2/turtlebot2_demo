/**
 * @file /src/lib/priority_win.cpp
 *
 * @brief Posix priority configuration implementation.
 *
 * @date April 2013
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_WIN32)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/threads/priority_win.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using std::ostringstream;

/*****************************************************************************
** Implementation [Process]
*****************************************************************************/

bool set_priority(Priority priority_level) ecl_debug_throw_decl(StandardException)
{
    if (priority_level >= RealTimePriority1) {
    	return threads::set_real_time_priority(0, priority_level);
    }

    BOOL bResult = FALSE;
    HANDLE hThread = GetCurrentThread();

    if (!hThread) {
    	ecl_debug_throw(threads::throwPriorityException(LOC));
    	return false;
    }

    switch (priority_level) {
	case CriticalPriority:
		bResult = SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);
		break;
	case HighPriority:
		bResult = SetThreadPriority(hThread, THREAD_PRIORITY_ABOVE_NORMAL);
		break;
	case NormalPriority:
		bResult = SetThreadPriority(hThread, THREAD_PRIORITY_NORMAL);
		break;
	case LowPriority:
		bResult = SetThreadPriority(hThread, THREAD_PRIORITY_BELOW_NORMAL);
		break;
	case BackgroundPriority:
		bResult = SetThreadPriority(hThread, THREAD_PRIORITY_IDLE);
		break;
	default:
		break;
    }

    if (!bResult) {
        ecl_debug_throw(threads::throwPriorityException(LOC));
        return false;
    }

    return true;
}

Priority get_priority() ecl_debug_throw_decl(StandardException)
{
    HANDLE hThread = GetCurrentThread();

    if (!hThread) {
        ecl_debug_throw(threads::throwPriorityException(LOC));
        return UnknownPriority;
    }

    DWORD dwPriority = GetThreadPriority(hThread);

    switch (dwPriority) {
    case THREAD_PRIORITY_TIME_CRITICAL:
    	return RealTimePriority1; // representative of real-time priority group
    case THREAD_PRIORITY_HIGHEST:
    	return CriticalPriority;
    case THREAD_PRIORITY_ABOVE_NORMAL:
    	return HighPriority;
    case THREAD_PRIORITY_NORMAL:
    	return NormalPriority;
    case THREAD_PRIORITY_BELOW_NORMAL:
    	return LowPriority;
    case THREAD_PRIORITY_IDLE:
    	return BackgroundPriority;
    default:
    	break;
    }

    return UnknownPriority;
}

/*****************************************************************************
** Implementation [Debugging]
*****************************************************************************/

std::string print_priority_diagnostics() ecl_debug_throw_decl(StandardException) {

    ostringstream ostream;

    ostream << "\n";
    ostream << "***********************************************************\n";
    ostream << "*                 Priority Statistics\n";
    ostream << "***********************************************************\n";
    ostream << "\n";
    switch ( get_priority() ) {
        case ( BackgroundPriority ) : {
            ostream << "Priority...................................Background\n";
            break;
        }
        case ( LowPriority ) : {
            ostream << "Priority...................................Low\n";
            break;
        }
        case ( NormalPriority ) : {
            ostream << "Priority...................................Normal\n";
            break;
        }
        case ( HighPriority ) : {
            ostream << "Priority...................................High\n";
            break;
        }
        case ( CriticalPriority ) : {
            ostream << "Priority...................................Critical\n";
            break;
        }
        case ( RealTimePriority4 ) : {
            ostream << "Priority...................................RealTime4\n";
            break;
        }
        case ( RealTimePriority3 ) : {
            ostream << "Priority...................................RealTime3\n";
            break;
        }
        case ( RealTimePriority2 ) : {
            ostream << "Priority...................................RealTime2\n";
            break;
        }
        case ( RealTimePriority1 ) : {
            ostream << "Priority...................................RealTime1\n";
            break;
        }
        case ( DefaultPriority ) : {
            ostream << "Priority...................................Default (Inherited)\n";
            break;
        }
        case ( UnknownPriority ) : {
            ostream << "Priority...................................Unknown\n";
            break;
        }
    }
    return ostream.str();
}


/*****************************************************************************
** Hidden Implementations
*****************************************************************************/

namespace threads {

bool set_real_time_priority(int policy,int priority_level) ecl_debug_throw_decl(StandardException) {
	// policy is ignored

    HANDLE hThread = GetCurrentThread();

    if (!hThread) {
    	ecl_debug_throw(threads::throwPriorityException(LOC));
    	return false;
    }

	BOOL bResult = FALSE;

    if (priority_level >= RealTimePriority1) {
    	bResult = SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
    }

    return bResult != FALSE;
}
} // namespace threads
}; // namespace ecl

#endif /* ECL_IS_WIN32 */
