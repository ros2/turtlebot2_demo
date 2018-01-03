/**
 * @file /include/ecl/threads/threadable.hpp
 *
 * @brief Posix version of the threadable interfaces.
 *
 * Posix threadable functionality to be used as an inherited interface.
 * For a composited interface, use the Thread class.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_THREADABLE_POS_HPP_
#define ECL_THREADS_THREADABLE_POS_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/utilities/parameter.hpp>
#include <ecl/config/macros.hpp>
#include "thread.hpp"
#include "priority.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [Threadable]
*****************************************************************************/
/**
 * @brief An inheritable interface for starting threads.
 *
 * This concept is for worker threads that want to retain state information
 * (possibly for use by other parts of the program) in a class. This is often
 * a convenient structure for c++ oriented threaded programs.
 *
 * Implementation wise, it provides an inheritable interface for your
 * threading class. All the class needs to do is:
 *
 * - inherit the Threadable class.
 * - implement the runnable() method.
 * - call the start() method to begin running in a sepearate thread.
 *
 * Note that it will not spawn multiple threads - it has a check that ensures it will
 * only execute one thread at any point in time. It is designed to be something
 * more akin to a thread function object rather than a thread factory.
 *
 * However, once the threaded work is finished, you may start the worker
 * thread once more.
 *
 * @sa @ref ecl::Thread "Thread".
 */
class ECL_PUBLIC Threadable {
public:
	virtual ~Threadable() {}

	/**
	 * @brief Begin execution of the worker thread.
	 *
	 * This begins execution of the implemented virtual runnable() in a separate thread.
	 * Note that it makes a check first to see if a thread is already running and simply
	 * returns if there is. There can be only one!
	 *
	 * @return bool : true if started, false if a worker thread is already running.
	 */
	bool start(const Priority &priority = DefaultPriority) {
		if ( isRunning() ) { return false; }
		isRunning(true); // This is not ideal, as it's not perfectly atomic (i.e. someone can query isRunning before this gets set).
		Thread thread(&Threadable::executeThreadFunction,*this, priority);
		return true;
	}
	/**
	 * @brief Used to check if a worker thread is already running.
	 *
	 * Use this to check if a worker thread is already running.
	 */
	Parameter<bool> isRunning;

protected:
	Threadable() : isRunning(false) {}

	virtual void runnable() = 0;

private:
	void executeThreadFunction() {
		runnable();
		isRunning(false);
	}
};

} // namespace ecl

#endif /* ECL_IS_POSIX */
#endif /* ECL_THREADS_THREADABLE_POS_HPP_ */
