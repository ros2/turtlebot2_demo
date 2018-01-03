/**
 * @file /include/ecl/threads/thread_win.hpp
 *
 * @brief Win32 interface for the thread class.
 *
 * @date April 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_THREAD_WIN_HPP_
#define ECL_THREADS_THREAD_WIN_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_WIN32)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <windows.h>
#include "thread_exceptions_pos.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/nullary_function.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/utilities/void.hpp>
#include <ecl/utilities/function_objects.hpp>
#include "priority_win.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace threads {

/*****************************************************************************
** Interface [ThreadTask]
*****************************************************************************/
/**
 * @brief Psuedo-mechanism for creating ThreadTask objects in @ref ecl::Thread "Thread".
 *
 * Used by the thread class for creating ThreadTask objects on the heap. Not intended
 * for direct use.
 */
class ECL_LOCAL ThreadTaskBase {
public:
	virtual ~ThreadTaskBase() {};

protected:
	ThreadTaskBase(const Priority& priority) : priority_level(priority) {}; /**< This constructor is only enabled for child ThreadTask classes. **/
	ecl::Priority priority_level;
};
/**
 * @brief The thread task template family.
 *
 * Stores all the functional information needed by a posix thread to start, run and
 * carry through to completion. Used by the @ref ecl::Thread "Thread" class and
 * not intended for direct use.
 *
 * @tparam F : the nullary function object type to be run inside the thread.
 */
template <typename F, bool IsReferenceWrapper = false>
class ECL_LOCAL ThreadTask : public ThreadTaskBase {
public:
	/**
	 * @brief Initialises the class with a nullary function object.
	 *
	 * Initialises the class with a nullary function object.
	 *
	 * @param f : the nullary function.
	 * @param priority : the priority level for the thread task.
	 */
	ThreadTask(const F &f, const Priority &priority) : ThreadTaskBase(priority), function(f) {
		ecl_compile_time_concept_check(ecl::NullaryFunction<F>);
	};
	virtual ~ThreadTask() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief Win32 thread function wrapper.
	 *
	 * This static function enables member functions to be run by the CreateThread function.
	 * It's a simple trick that will automatically get used by the @ref ecl::Thread "Thread"
	 * class.
	 *
	 * @param ptr_this : a pointer to an instance of this class.
	 * @return unsigned int : a Win32 thread requirement, the return value.
	 */
	static unsigned int EntryPoint(void *ptr_this) {
	    ThreadTask< F, false > *ptr = static_cast< ThreadTask< F, false > * >(ptr_this);
	    (ptr->function)();
	    return 0;
	}

private:
	F function;
};

/**
 * @brief Specialisation of the thread tasks, this one handles reference wrappers to funciton objects.
 *
 * Stores all the referenced functional information needed by a posix thread to start, run and
 * carry through to completion. Used by the @ref ecl::Thread "Thread" class and
 * not intended for direct use.
 *
 * @tparam F : the nullary function object referenced and run inside the thread.
 */
template <typename F>
class ECL_LOCAL ThreadTask<F, true> : public ThreadTaskBase {
public:
	/**
	 * @brief Initialises the class with a nullary function object.
	 *
	 * Initialises the class with a nullary function object.
	 *
	 * @param f : the nullary function.
	 * @param priority : the priority level for the thread task.
	 */
	ThreadTask(const F &f, const Priority &priority) : ThreadTaskBase(priority), function(f.reference()) {
		ecl_compile_time_concept_check(ecl::NullaryFunction< typename F::type>);
	};
	virtual ~ThreadTask() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief Win32 thread function wrapper.
	 *
	 * This static function enables member functions to be run by the CreateThread function.
	 * It's a simple trick that will automatically get used by the @ref ecl::Thread "Thread" class
	 *
	 * @param ptr_this : a pointer to an instance of this class.
	 * @return unsigned int : a Win32 thread requirement. the return value.
	 */
	static unsigned int EntryPoint(void *ptr_this) {
	    ThreadTask< F, true > *ptr = static_cast< ThreadTask< F, true > * >(ptr_this);
	    (ptr->function)();
	    return 0;
	}

private:
	typename F::type &function;
};

}; // namespace threads

/*****************************************************************************
** Interface [Thread]
*****************************************************************************/
/**
 * @brief Win32 thread class.
 *
 * This is a simple one-shot thread class that uses the principle of RAII.
 * Once constructed, the thread immediately starts and any cleanup is taken
 * care of by the destructor.
 *
 * <b>Usage:</b>
 *
 * Initialise with a nullary function object or one of the convenience
 * constructors for global/static and member functions.
 *
 * @code
   using ecl::generateFunctionObject;

   int f() {}
   int g(int i) {}
   class A {
       void f() {}
       void g(int i) {}
   };
   class FunctionObject {
   public:
       typedef void result_type;
       void operator()() { //
       }
   };
   // ...
   A a;
   FunctionObject function_object;

   Thread thread1(f));                                   // Thread a nullary global function.
   Thread thread2(generateFunctionObject(g, 3));         // Thread a bound unary global function.
   Thread thread3(&A::f, a);     						 // Thread a nullary member function.
   Thread thread4(generateFunctionObject(&A::g, a, 2));  // Thread a bound unary member function.
   Thread thread5(function_object);                      // Thread a nullary function object.
   Thread thread6(ref(function_object));                 // Thread a reference to a nullary function object.
   @endcode

   Note that you must ensure that the instance (in the above example, 'a') must not go out
 * of scope while the thread is running. To keep it lightweight, it does not copy the class to the thread,
 * merely includes a reference to it.
 *
 * One important usage point, you may allow the thread instance to go out of
 * scope without affecting the created thread - this simply detaches the
 * thread so you can no longer administer it, but it will continue running
 * in the background.
 *
   @code
   void g() {
       for (int i = 0; i < 10; ++i ) {
           sleep(1);
           cout << i << endl;
       }
   }

   void create_out_of_scope_thread() {
       Thread thread(g);
   } // thread will go out of scope here.
   // ...
   create_out_of_scope_thread();
   // Cannot adminster the thread from here, but it will continue running.
   sleep(10); // Note that we have no way of joining with it.
   @endcode

   <b>Error Handling</b>

   These threads are RAII style - error handling is done with the constructor which will throw exceptions
   in debug mode only (too much of a slowdown to do so in release mode if many threads are required).

   If exceptions are turned off or you are in release mode, isRunning() can be checked to verify that
   the thread is running as expected.
 */
class ecl_threads_PUBLIC Thread {
public:
	/**
	 * @brief Default constructor that doesn't automatically start the thread.
	 *
	 * This is useful sometimes when you need to delay the start of the thread
	 * execution (to wait on some initialisation parameters for example).
	 * Use one of the start() functions to kick the thread off.
	 */
	Thread() :
		thread_handle(NULL),
		thread_task(NULL),
		has_started(false),
		join_requested(false)
	{}
	/**
	 * @brief Convenience constructor that starts a new thread with a void global/static function.
	 *
	 * This provides an easy construction format for void global/static functions.
	 * It essentially emulates the call made by the more general function
	 * object constructor:
	 *
	 * <b>Usage:</b>
	 * @code
	 * void f() {}
	 * // ...
	 * Thread thread(f);
	 * @endcode
	 *
	 * @param function : a void function pointer, void (*)().
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : no. of bytes to allocate on the stack (default is to use the system value, usually 8k).
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 */
	Thread(VoidFunction function, const Priority &priority = DefaultPriority, const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Starts the thread if not already started.
	 *
	 * @param function : a void function pointer, void (*)().
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : ignored, currently not available.
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 * @return Error : error result, fallback for when exceptions aren't available.
	 */
	Error start(VoidFunction function, const Priority &priority = DefaultPriority, const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Convenience method that starts a new thread utilising a void member function.
	 *
	 * This provides an easy construction format for void member functions.
	 * It essentially emulates the call made by the more general function
	 * object constructor:
	 *
	 * <b>Usage:</b>
	 * @code
	 * class A {
	 *     void f() {}
	 * };
	 * // ...
	 * A a;
	 * Thread thread(&A::f, a);
	 * @endcode
	 *
	 * @param function : the member function.
	 * @param c : the member function's class instance.
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : ignored, currently not available.
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 */
	template <typename C>
	Thread(void (C::*function)(), C &c, const Priority &priority = DefaultPriority,  const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Starts the thread if not already started.
	 *
	 * @param function : the member function.
	 * @param c : the member function's class instance.
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : ignored, currently not available.
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 * @return Error : error result, fallback for when exceptions aren't available.
	 */
	template <typename C>
	Error start(void (C::*function)(), C &c, const Priority &priority = DefaultPriority,  const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);

	/**
	 * @brief Starts a new thread utilising a nullary function object.
	 *
	 * Starts a new thread with the specified nullary function object. You can create your
	 * own custom nullary functions, or utilise the <i>generateFunctionObject</i> method from ecl_utilities
	 * to bind/generate them.
	 *
	 * <b>Usage:</b>
	   @code
	   int f(int i) {}
	   class A {
	       void f() {}
	       void g(int i) {}
	   };
	   class FunctionObject {
	   public:
		   typedef void result_type;
		   void operator()() { //
		   }
	   };
	   // ...
	   A a;
	   FunctionObject function_object;
	   Thread thread0(generateFunctionObject(f, 3));         // Bind the first argument to a global function
	   Thread thread1(generateFunctionObject(&A::f, a));     // Generate a typical member function
	   Thread thread2(generateFunctionObject(&A::g, a, 2));  // Bind the first argument to a member function
	   Thread thread3(function_object);                      // Nullary function object (uses operator()())
       Thread thread4(ref(function_object));                 // Thread a reference to a nullary function object.
	   @endcode
	 *
	 * This will emit a compile time failure if the template argument does
	 * not conform to the <i>NullaryFunction</i> concept (refer to the ecl_concepts package).
	 *
	 * @param function : the nullary function object.
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : ignored, currently not available.
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 */
	template <typename F>
	Thread(const F &function, const Priority &priority = DefaultPriority,  const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Starts a new thread utilising a nullary function object.
	 *
	 * @param function : the nullary function object.
	 * @param priority : set the priority level for the thread.
	 * @param stack_size : ignored, currently not available.
	 * @exception StandardException : throws if thread creation fails [debug mode only].
	 * @return Error : error result, fallback for when exceptions aren't available.
	 */
	template <typename F>
	Error start(const F &function, const Priority &priority = DefaultPriority,  const long &stack_size = -1 ) ecl_debug_throw_decl(StandardException);

	/**
	 * @brief Cleans up the resources allocated to the thread.
	 *
	 * If not already joined, this ensures the thread resources are cleaned up when the thread
	 * object goes out of scope. Note that this will not terminate the actual running thread (
	 * if the running thread is actually still running), rather it simply detaches and loses
	 * administrative control over it.
	 */
	virtual ~Thread();

	/**
	 * @brief Check to see if the thread is still running.
	 *
	 * @return bool : true if the thread task is still running, false otherwise.
	 */
	bool isRunning() { if ( thread_task == NULL ) { return false; } else { return true; } }

	/**
	 * @brief Queue a cancel request for this thread to abort.
	 *
	 * This commands sends a request to the thread for cancellation. The calling thread
	 * will not wait, it merely queues the request and then keeps processing. The running
	 * thread will also keep processing, but will cancel as soon as it reaches its next
	 * cancellation point.
	 *
	 * Threads can be setup to deny cancel requests, but this class always configures them as
	 * cancellable.
	 *
	 * @exception StandardException : throws if this thread is no longer running [debug mode only].
	 */
	void cancel() ecl_debug_throw_decl(StandardException);

	/**
	 * @brief Join with this thread.
	 *
	 * This command causes the calling thread to halt up and wait for the running thread to
	 * finish.
	 * @exception StandardException : throws if this thread is no longer running [debug mode only].
	 */
	void join() ecl_assert_throw_decl(StandardException);


private:
	HANDLE thread_handle;
    threads::ThreadTaskBase *thread_task;
    bool has_started;
    bool join_requested;

	void initialise(const long &stack_size) ecl_assert_throw_decl(StandardException);

	enum ThreadProperties {
		DefaultStackSize = -1
	};
};

/*****************************************************************************
** Template Implementation [Thread]
*****************************************************************************/

template <typename C>
Thread::Thread(void (C::*function)(), C &c, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException) :
	thread_handle(NULL),
	thread_task(NULL),
	has_started(false),
	join_requested(false)
{
	start<C>(function, c, priority, stack_size);

}

template <typename F>
Thread::Thread(const F &function, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException) :
	thread_handle(NULL),
	thread_task(NULL),
	has_started(false),
	join_requested(false)
{
	start<F>(function, priority, stack_size);
}

template <typename C>
Error Thread::start(void (C::*function)(), C &c, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException)
{
	// stack_size is ignored

	if ( has_started ) {
		ecl_debug_throw(StandardException(LOC,BusyError,"The thread has already been started."));
		return Error(BusyError); // if in release mode, gracefully fall back to return values.
	} else {
		has_started = true;
	}

	thread_task = new threads::ThreadTask< BoundNullaryMemberFunction<C,void> >(generateFunctionObject( function, c ), priority);

    DWORD threadid;

    thread_handle = CreateThread(NULL,
    	0,
    	(LPTHREAD_START_ROUTINE)threads::ThreadTask< BoundNullaryMemberFunction<C,void> >::EntryPoint,
    	thread_task,
    	0,
    	&threadid);

    if (!thread_handle) {
    	ecl_debug_throw(StandardException(LOC, UnknownError, "Failed to create thread."));
    	return Error(UnknownError);
    }

    BOOL bResult = FALSE;

    if (priority >= RealTimePriority1) {
    	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_TIME_CRITICAL);
    }

    switch (priority) {
        case CriticalPriority:
        	bResult = SetThreadPriority(thread_handle, HIGH_PRIORITY_CLASS);
            break;
        case HighPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_ABOVE_NORMAL);
            break;
        case LowPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_BELOW_NORMAL);
            break;
        case BackgroundPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_IDLE);
            break;
        default:
            break;
    }

    if (!bResult) {
        ecl_debug_throw(threads::throwPriorityException(LOC));
    }

    return Error(NoError);
}

template <typename F>
Error Thread::start(const F &function, const Priority &priority, const long &stack_size) ecl_debug_throw_decl(StandardException)
{
	// stack_size is ignored

	if ( has_started ) {
		ecl_debug_throw(StandardException(LOC,BusyError,"The thread has already been started."));
		return Error(BusyError); // if in release mode, gracefully fall back to return values.
	} else {
		has_started = true;
	}

	thread_task = new threads::ThreadTask<F, is_reference_wrapper<F>::value >(function, priority);

    DWORD threadid;

    thread_handle = CreateThread(NULL,
    	0,
    	(LPTHREAD_START_ROUTINE)threads::ThreadTask<F, is_reference_wrapper<F>::value >::EntryPoint,
    	thread_task,
    	0,
    	&threadid);

    if (!thread_handle) {
    	ecl_debug_throw(StandardException(LOC, UnknownError, "Failed to create thread."));
    	return Error(UnknownError);
    }

    BOOL bResult = FALSE;

    if (priority >= RealTimePriority1) {
    	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_TIME_CRITICAL);
    }

    switch (priority) {
        case CriticalPriority:
        	bResult = SetThreadPriority(thread_handle, HIGH_PRIORITY_CLASS);
            break;
        case HighPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_ABOVE_NORMAL);
            break;
        case LowPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_BELOW_NORMAL);
            break;
        case BackgroundPriority:
        	bResult = SetThreadPriority(thread_handle, THREAD_PRIORITY_IDLE);
            break;
        default:
            break;
    }

    if (!bResult) {
        ecl_debug_throw(threads::throwPriorityException(LOC));
    }

    return Error(NoError);
}

}; // namespace ecl

#endif /* ECL_IS_WIN32 */
#endif /* ECL_THREADS_THREAD_WIN_HPP_ */
