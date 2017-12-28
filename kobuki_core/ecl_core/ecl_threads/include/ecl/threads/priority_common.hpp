/**
 * @file /include/ecl/threads/priority_common.hpp
 *
 * @brief Common types for priority scheduling.
 *
 * @date January 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_PRIORITY_COMMON_HPP_
#define ECL_THREADS_PRIORITY_COMMON_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Common types
*****************************************************************************/

/**
 * @brief Shared abstraction of the scheduling priorities.
 *
 * Defines a priority level for priority scheduling of a thread/process.
 * They are effectively ranked as indicated, however their
 * implementation will be different for different systems.
 */
enum Priority {
    DefaultPriority = 0,
    UnknownPriority,
    BackgroundPriority,
    LowPriority,
    NormalPriority,
    HighPriority,
    CriticalPriority,
    RealTimePriority1,
    RealTimePriority2,
    RealTimePriority3,
    RealTimePriority4,
};


} // namespace ecl

#endif /*ECL_THREADS_PRIORITY_COMMON_HPP_*/
