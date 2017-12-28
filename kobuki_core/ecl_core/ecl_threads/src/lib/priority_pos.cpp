/**
 * @file /src/lib/priority_pos.cpp
 *
 * @brief Posix priority configuration implementation.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <unistd.h>
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/threads/priority_pos.hpp"

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
    /*************************************************************************
     * Real time priority exception. Run this with absolute priority rather
     * than the 'niceness' values. We use Round Robin scheduling here, the
     * alternative is Fifo so we can get some automatic scheduling if we
     * want it.
     *
     * Priority levels usually range from 1 to 99, but we map them virtually
     * to system built-ins just in case. The virtual map only uses a few
     * levels of priority as shown below.
     *
     * RealTimePriority1 = priority_min [lowest]
     * RealTimePriority2 = priority_min + priority_range/10
     * RealTimePriority3 = priority_min + 2*priority_range/10
     * RealTimePriority4 = priority_min + 3*priority_range/10 [highest]
     *
    *************************************************************************/
    if ( priority_level >= RealTimePriority1 ) {
        #if _POSIX_PRIORITY_SCHEDULING > 0
            int rr_min = sched_get_priority_min(SCHED_RR); int rr_max = sched_get_priority_max(SCHED_RR);
            if ( ( rr_min == -1 ) || (rr_max == -1) ) {
                ecl_throw(StandardException(LOC,NotSupportedError,"The posix SCHED_RR policy is not available on this system [sched_get_priority_min/max]."));
                return false;
            }
            ecl_try {
            	// usually exception will put it into a catch, otherwise we return false if it fails.
                if ( !threads::set_real_time_priority(SCHED_RR,rr_min+(priority_level - RealTimePriority1)*(rr_max - rr_min)/10) ) {
                	return false;
                }
            } ecl_catch(StandardException &e ) {
                ecl_throw(StandardException(e)); // Rethrow
            }
        #else
            ecl_throw(StandardException(LOC,NotSupportedError,"Your version of posix does not support real time priority scheduling for process management."));
            return false;
        #endif
        return true;
    }

    /*************************************************************************
    ** Regular priority levels using 'niceness'. Lower values usually give
    ** the process higher importance (-20 to 20 usually). This is an
    ** indicator to the scheduler on how to bias the process's dynamic
    ** scheduling priority which can be seen when you do a 'top'.
    *************************************************************************/
    int return_value = 0;

    switch (priority_level) {
        case CriticalPriority : {
            return_value = setpriority(PRIO_PROCESS,0,PRIO_MIN);
            break;
        }
        case HighPriority : {
            setpriority(PRIO_PROCESS,0,PRIO_MIN + (PRIO_MAX-PRIO_MIN)/4);
            break;
        }
        case NormalPriority : {
            setpriority(PRIO_PROCESS,0,PRIO_MIN + (PRIO_MAX-PRIO_MIN)/2);
            break;
        }
        case LowPriority : {
            setpriority(PRIO_PROCESS,0,PRIO_MIN + 3*(PRIO_MAX-PRIO_MIN)/4);
            break;
        }
        case BackgroundPriority : {
            setpriority(PRIO_PROCESS,0,PRIO_MAX);
            break;
        }
        default : {
            // RealTimePriority# already handled, Default and Unknown -> do nothing.
            break;
        }
    }
    if ( return_value == -1 ) {
        ecl_debug_throw(threads::throwPriorityException(LOC));
        return false;
    }
    return true;
}

Priority get_priority() ecl_debug_throw_decl(StandardException)
{
    /******************************************
    ** Check for Scheduling Policy (Trad/RT)
    *******************************************/
    #if _POSIX_PRIORITY_SCHEDULING > 0
        int scheduler = sched_getscheduler(0);
        switch ( scheduler ) {
            case ( -1 ) : {
                ecl_debug_throw(threads::throwPriorityException(LOC));
                return UnknownPriority;
                break;
            }
            case ( SCHED_OTHER ) : {
                // We just want the niceness, get it outside of this switch.
                break;
            }
            case ( SCHED_RR ) : { // Realtime priorities.
                /******************************************
                ** Check RealTime Priority Level
                *******************************************/
                sched_param param;
                if ( sched_getparam(0,&param) != 0 ) {
                    ecl_debug_throw(threads::throwPriorityException(LOC));
                    return UnknownPriority;
                }
                int rr_min = sched_get_priority_min(SCHED_RR);
                int rr_max = sched_get_priority_max(SCHED_RR);
                if ( ( rr_min == -1 ) || (rr_max == -1) ) {
                    ecl_throw(StandardException(LOC,NotSupportedError,"The posix SCHED_RR policy is not available on this system [sched_get_priority_min/max]."));
                    return UnknownPriority;
                }
                if ( param.sched_priority >= rr_min + 3*(rr_max-rr_min)/10 ) {
                    return RealTimePriority4;
                } else if ( param.sched_priority >= rr_min + 2*(rr_max-rr_min)/10 ) {
                    return RealTimePriority3;
                } else if ( param.sched_priority >= rr_min + (rr_max-rr_min)/10 ) {
                    return RealTimePriority2;
                } else {
                    return RealTimePriority1;
                }
                break;
            }
            case ( SCHED_FIFO ) : { return UnknownPriority; } // We dont use this one.
			#if defined(SCHED_BATCH) // Didn't turn up on an old gcc3 with an arm pax270 board.
            	case ( SCHED_BATCH ) : { return UnknownPriority; } // We dont use this one.
			#endif
            default : { return UnknownPriority; } // Shouldn't ever reach here, but we might pick up new policies that we dont use here.
        }
    #endif
    /******************************************
    ** Check traditional priority niceness
    *******************************************/
    switch ( getpriority(PRIO_PROCESS,0) ) {
        case (PRIO_MIN) : { return CriticalPriority; }
        case (PRIO_MIN + (PRIO_MAX-PRIO_MIN)/4) : { return HighPriority; }
        case (PRIO_MIN + (PRIO_MAX-PRIO_MIN)/2) : { return NormalPriority; }
        case (PRIO_MIN + 3*(PRIO_MAX-PRIO_MIN)/4) : { return LowPriority; }
        case (PRIO_MAX) : { return BackgroundPriority; }
        default : return NormalPriority;
    }

}
/*****************************************************************************
** Implementation [Debugging]
*****************************************************************************/

std::string print_priority_diagnostics() ecl_debug_throw_decl(StandardException) {

    ostringstream ostream;

    ostream << "\n";
    ostream << "***********************************************************\n";
    ostream << "*                  System Statistics\n";
    ostream << "***********************************************************\n";
    ostream << "\n";

	#if _POSIX_PRIORITY_SCHEDULING > 0
		int rr_min = sched_get_priority_min(SCHED_RR);
		int rr_max = sched_get_priority_max(SCHED_RR);
		if ( ( rr_min == -1 ) || (rr_max == -1) ) {
			ecl_throw(StandardException(LOC,NotSupportedError,"The posix SCHED_RR policy is not available on this system [sched_get_priority_min/max]."));
			return std::string("The posix SCHED_RR policy is not available on this system [sched_get_priority_min/max].");
		}
		ostream << "Real Time Priorities [Low,High]............[" << rr_min << "," << rr_max << "]\n";
	#endif
    ostream << "Niceness [Low,High]........................[" << PRIO_MAX << "," << PRIO_MIN << "]\n";
    ostream << "\n";
    ostream << "***********************************************************\n";
    ostream << "*                 Priority Statistics\n";
    ostream << "***********************************************************\n";
    ostream << "\n";
    #if _POSIX_PRIORITY_SCHEDULING > 0
		int scheduler = sched_getscheduler(0);
		switch ( scheduler ) {
			case ( -1 ) : {
				ecl_debug_throw(threads::throwPriorityException(LOC));
				return std::string("Call to sched_getscheduler failed.");
			}
			case ( SCHED_OTHER ) : {
				ostream << "Scheduler..................................SCHED_OTHER" << "\n";
				break;
			}
			case ( SCHED_RR ) : {
				ostream << "Scheduler..................................SCHED_RR [RT]" << "\n";
				break;
			}
			case ( SCHED_FIFO ) : {
				ostream << "Scheduler..................................SCHED_FIFO [RT]" << "\n";
				break;
			}
#if defined(SCHED_BATCH)
			case ( SCHED_BATCH ) : {
				ostream << "Scheduler..................................SCHED_BATCH" << "\n";
				break;
			}
#endif
			default : {
				ostream << "Scheduler..................................Unknown" << "\n";
				break;
			}
		}
    #endif
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

	#if _POSIX_PRIORITY_SCHEDULING > 0
		ostringstream ostream;

		/*********************
		** Exception Throwing
		**********************/
		if ( priority_level < sched_get_priority_min(policy) ) {
			ostream << "The realtime process priority requested was smaller than the minimum value permitted[";
			ostream << sched_get_priority_min(policy) << "]\n";
			ecl_throw(StandardException(LOC,OutOfRangeError, ostream.str()));
			return false;
		} else if (priority_level > sched_get_priority_max(policy) ) {
			ostream << "The realtime process priority requested was greater than the maximum value permitted[";
			ostream << sched_get_priority_max(policy) << "]\n";
			ecl_throw(StandardException(LOC,OutOfRangeError, ostream.str()));
			return false;
		}

		/*********************
		** Configuration
		**********************/
		sched_param schedule_parameters;
		schedule_parameters.sched_priority = priority_level;
		if ( sched_setscheduler(0, policy, &schedule_parameters) == -1 ) {
			ecl_debug_throw(throwPriorityException(LOC));
			return false;
		}
		return true;
	#else
        ecl_throw(StandardException(LOC,ecl::NotSupportedError,"Your version of posix does not support real time priority scheduling for process management."));
        return false;
	#endif
}
} // namespace threads
}; // namespace ecl

#endif /* ECL_IS_POSIX */
