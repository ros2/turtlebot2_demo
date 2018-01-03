/**
 * @file /src/lib/thread_pos.cpp
 *
 * @brief Posix thread implementation.
 *
 * @date June 2009
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
#include "../../include/ecl/threads/thread_pos.hpp"
#include <sys/resource.h> // provides PRIO_MIN, PRIO_MAX for niceness levels

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
* Thread Class Methods
*****************************************************************************/

Thread::Thread(VoidFunction function, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException) :
	thread_task(NULL),
	has_started(false),
	join_requested(false)
{
	start(function, priority, stack_size);
}

Error Thread::start(VoidFunction function, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException)
{
	if ( has_started ) {
		ecl_debug_throw(StandardException(LOC,BusyError,"The thread has already been started."));
		return Error(BusyError); // if in release mode, gracefully fall back to return values.
	} else {
		has_started = true;
	}
	initialise(stack_size);
	NullaryFreeFunction<void> nullary_function_object = generateFunctionObject(function);
	thread_task = new threads::ThreadTask< NullaryFreeFunction<void> >(nullary_function_object, priority);
    int result = pthread_create(&(this->thread_handle), &(this->attrs), threads::ThreadTask< NullaryFreeFunction<void> >::EntryPoint, thread_task);
	pthread_attr_destroy(&attrs);
    if ( result != 0 ) {
    	delete thread_task;
    	thread_task = NULL;
    	ecl_debug_throw(threads::throwPthreadCreateException(LOC,result));
		return threads::handlePthreadCreateError(result); // if in release mode, gracefully fall back to return values.
    }
    return Error(NoError);
}

Thread::~Thread() {
	// This should work, but it doesn't. pthread_tryjoin_np fails to differentiate
	// between an already running thread and one that has already been joined (both
	// return EBUSY instead of EBUSY and EINVAL
//	int res;
//	if ( (res = pthread_tryjoin_np(thread_handle,0)) != 0 ) {
//		if ( res == EBUSY ) {
//			// still running, so detach.
//		    pthread_detach(thread_handle);
//		} else {
//			// Has already been joined, so leave it be.
//		}
//	}
	if ( has_started && !join_requested ) {
	    pthread_detach(thread_handle);
	}
}
void Thread::cancel() ecl_debug_throw_decl(StandardException) {
	int result = pthread_cancel(thread_handle);
	// Note - if we reach here, then entrypoint hasn't gone through to conclusion and
	// subsequently the thread_task object on the heap hasn't been deleted. So...
	// always delete it here!
    if ( thread_task != NULL ) {
    	delete thread_task;
    	thread_task = NULL;
    }
    if ( result != 0 ) {
        ecl_debug_throw(threads::throwPthreadJoinException(LOC,result));
    }
}

void Thread::join() ecl_debug_throw_decl(StandardException) {
	join_requested = true;
	if( thread_task != NULL ) {
		int result = pthread_join( thread_handle, 0 ); // This also frees up memory like pthread_detach
	    ecl_assert_throw( result == 0, threads::throwPthreadJoinException(LOC,result));
	}
}

void Thread::initialise(const long &stack_size) ecl_assert_throw_decl(StandardException) {

	pthread_attr_init( &attrs );
    /*************************************************************************
    ** Linux implementation does not allow PTHREAD_SCOPE_PROCESS
    *************************************************************************/
    pthread_attr_setscope(&attrs, PTHREAD_SCOPE_SYSTEM);
    // We can't set priorities for SCHED_OTHER (non real time) here anyway, so just use the default
    pthread_attr_setinheritsched(&attrs,PTHREAD_INHERIT_SCHED);

    /*************************************************************************
    ** Detached State
    **************************************************************************
    ** By default on linux, it is joinable. I do cleanup operations within
    ** this class, so no need to create it as detached, especially as more
    ** often than not, I need to be able to join to the thread again.
    *************************************************************************/
    pthread_attr_setdetachstate(&attrs,PTHREAD_CREATE_JOINABLE);

    /*************************************************************************
    ** Stack Size
    **************************************************************************/
    if ( stack_size != DefaultStackSize ) {
    	int result = pthread_attr_setstacksize(&attrs,stack_size);
		ecl_assert_throw( result == 0, StandardException(LOC,ConfigurationError,"Specified stack size was less than PTHREAD_STACK_MIN or wasn't a multiple of the page size."));
    }
}

}; // namespace ecl

#endif /* ECL_IS_POSIX */
