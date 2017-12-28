/**
 * @file src/utilities/detect_posix.cpp
 *
 * @brief Detects the posix functionality available on this platform.
 *
 * Does a series of macro checks to determine the level of posix
 * functionality that is available on this platform.
 *
 * @date May, 2009.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>

/*****************************************************************************
** Preprocessing
*****************************************************************************/

#ifdef __GNUC__
    #include <unistd.h>
	#include <cstdio>
#endif

/*****************************************************************************
** Main
*****************************************************************************/


int main()
{
#ifndef _POSIX_VERSION
   printf("No support for POSIX on this system!\n");
#else     //see what POSIX version
   long val;
   int errno;

   #if defined(POSIX_VERSION)
     printf("POSIX_VERSION = %d.\n",_POSIX_VERSION);
   #endif
   #if defined(POSIX2_VERSION)
     printf("POSIX2_VERSION = %d.\n",_POSIX2_VERSION);
   #endif
   #if defined(POSIX2_C_VERSION)
     printf("POSIX2_C_VERSION = %d.\n",_POSIX2_C_VERSION);
   #endif
   #if _POSIX_VERSION == 199009
      printf("This system supports POSIX.1 without support for POSIX.4.\n");
   #else
      #if _POSIX_VERSION >= 199309
         printf ("This system has Posix Version %ld. \n", _POSIX_VERSION);
         printf ("POSIX.1 and POSIX.4 are both supported. \n");
      #else
         printf("This system supports a strange POSIX version between 199009 and 199309.\n");
      #endif
    //DO POSIX.4 tests
      printf ("POSIX.4 Options Test Results:\n");
      #ifdef _POSIX_REALTIME_SIGNALS
         printf("   POSIX.4 Additional real time signals are supported.\n");
         #ifdef _POSIX_RTSIG_MAX
            printf("     _POSIX_RTSIG_MAX=%d Max real time signals.\n",_POSIX_RTSIG_MAX);
         #else
            printf("     No _POSIX_RTSIG_MAX value exists.\n");
         #endif
         #ifdef _POSIX_SIGQUEUE_MAX
            printf("     _POSIX_SIGQUEUE_MAX=%d Max real time signals at once per process.\n",_POSIX_SIGQUEUE_MAX);
         #else
            printf("     No _POSIX_SIGQUEUE_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Additional real time signals are not supported.\n");
      #endif
      #ifdef _POSIX_PRIORITY_SCHEDULING
         printf("   POSIX.4 Priority scheduling is supported.\n");
      #else
         printf("   POSIX.4 Priority scheduling is not supported.\n");
      #endif
      #ifdef _POSIX_TIMERS
         printf("   POSIX.4 Clocks and timers are supported.\n");
         #ifdef _POSIX_TIMER_MAX
            printf("     _POSIX_TIMER_MAX=%d Max number of concurrent timers a process can have.\n",_POSIX_TIMER_MAX);
         #else
            printf("     No _POSIX_TIMER_MAX value exists.\n");
         #endif
         #ifdef _POSIX_DELAYTIMER_MAX
            printf("     _POSIX_DELAYTIMER_MAX=%d Max number of times a timer can have a detectable overrun.\n",_POSIX_DELAYTIMER_MAX);
         #else
            printf("     No _POSIX_DELAYTIMER_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Clocks and timers are not supported.\n");
      #endif
      #ifdef _POSIX_ASYNCHRONOUS_IO
         printf("   POSIX.4 Asynchronous I/O is supported.\n");
         #ifdef _POSIX_AIO_LISTIO_MAX
            printf("     _POSIX_AIO_LISTIO_MAX=%d Max operations in one call\n",_POSIX_AIO_LISTIO_MAX);
         #else
            printf("     No _POSIX_AIO_LISTIO_MAX value exists.\n");
         #endif
         #ifdef _POSIX_AIO_MAX
            printf("     _POSIX_AIO_MAX=%d Max concurrent Async I/Os\n",_POSIX_AIO_MAX);
         #else
            printf("     No _POSIX_AIO_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Asynchronous I/O is not supported.\n");
      #endif
      #ifdef _POSIX_SYNCHRONIZED_IO  //Only supported if Asynchronous I/O is supported
         printf("   POSIX.4 Synchronized I/O is supported.\n");
      #else
         printf("   POSIX.4 Synchronized I/O is not supported.\n");
      #endif
      #ifdef _POSIX_PRIORITIZED_IO
         printf("   POSIX.4 Prioritized asynchronous I/O is supported.\n");
         #ifdef _POSIX_AIO_PRIO_DELTA_MAX
            printf("     _POSIX_AIO_PRIO_DELTA_MAX=%d Max amount AIO priority can be decreased.\n",_POSIX_AIO_PRIO_DELTA_MAX);
         #else
            printf("     No _POSIX_AIO_PRIO_DELTA_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Prioritized asynchronous I/O is not supported.\n");
      #endif
      #ifdef _POSIX_FSYNC
         printf("   POSIX.4 The fsync function is supported.\n");
      #else
         printf("   POSIX.4 The fsync function is not supported.\n");
      #endif
      #ifdef _POSIX_MAPPED_FILES
         printf("   POSIX.4 Mapping files as memory is supported.\n");
      #else
         printf("   POSIX.4 Mapping files as memory is not supported.\n");
      #endif
      #ifdef _POSIX_MEMLOCK
         printf("   POSIX.4 Locking memory is supported.\n");
      #else
         printf("   POSIX.4 Locking memory is not supported.\n");
      #endif
      #ifdef _POSIX_MEMLOCK_RANGE
         printf("   POSIX.4 Locking memory ranges is supported.\n");
      #else
         printf("   POSIX.4 Locking memory ranges is not supported.\n");
      #endif
      #ifdef _POSIX_MEMORY_PROTECTION
         printf("   POSIX.4 Setting memory protection is supported.\n");
      #else
         printf("   POSIX.4 Setting memory protection is not supported.\n");
      #endif
      #ifdef _POSIX_MESSAGE_PASSING
         printf("   POSIX.4 Message Queues are supported.\n");
         #ifdef _POSIX_MQ_OPEN_MAX
            printf("     _POSIX_MQ_OPEN_MAX=%d Max # of message queues per process.\n",_POSIX_MQ_OPEN_MAX);
         #else
            printf("     No _POSIX_MQ_OPEN_MAX value exists.\n");
         #endif
         #ifdef _POSIX_MQ_PRIO_MAX
            printf("     _POSIX_MQ_PRIO_MAX=%d Max # of message priorities.\n",_POSIX_MQ_PRIO_MAX);
         #else
            printf("     No _POSIX_MQ_PRIO_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Message Queues are not supported.\n");
      #endif
      #ifdef _POSIX_SEMAPHORES
         printf("   POSIX.4 Semaphores are supported.\n");
         #ifdef _POSIX_SEM_NSEMS_MAX
            printf("     _POSIX_SEM_NSEMS_MAX=%d Max # of open semaphores per process.\n",_POSIX_SEM_NSEMS_MAX);
         #else
            printf("     No _POSIX_SEM_NSEMS_MAX value exists.\n");
         #endif
         #ifdef _POSIX_SEM_VALUE_MAX
            printf("     _POSIX_SEM_VALUE_MAX=%d Maximum semaphore value.\n",_POSIX_SEM_VALUE_MAX);
         #else
            printf("     No _POSIX_SEM_VALUE_MAX value exists.\n");
         #endif
      #else
         printf("   POSIX.4 Semaphores are not supported.\n");
      #endif
      #ifdef _POSIX_SHARED_MEMORY_OBJECTS
         printf("   POSIX.4 Shared memory objects are supported.\n");
      #else
         printf("   POSIX.4 Shared memory objects are not supported.\n");
      #endif

      #ifdef _POSIX_THREADS
         printf("   POSIX.1c pthreads are supported.\n");
      #else
         printf("   POSIX.1c pthreads are not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_ATTR_STACKADDRTHREAD_ATTR_STACKADDR
         printf("   POSIX.4 Thread stack address attribute option is supported.\n");
      #else
         printf("   POSIX.4 Thread stack address attribute option is not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_ATTR_STACKSIZE
         printf("   POSIX.4 Thread stack size attribute option is supported.\n");
      #else
         printf("   POSIX.4 Thread stack size attribute option is not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_SAFE_FUNCTIONS
         printf("   POSIX.4 Thread-safe functions are supported.\n");
      #else
         printf("   POSIX.4 Thread-safe functions are not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_PRIORITY_SCHEDULING
         printf("       POSIX.1c thread execution scheduling is supported.\n");
      #else
         printf("       POSIX.1c thread execution scheduling is not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_PRIO_INHERIT
         printf("   POSIX.4 Thread priority inheritance option is supported.\n");
      #else
         printf("   POSIX.4 Thread priority inheritance option is not supported.\n");
      #endif
      #ifdef _POSIX_THREAD_PROCESS_SHARED
         printf("   POSIX.4 Process-shared synchronization is supported.\n");
      #else
         printf("   POSIX.4 Process-shared synchronization is not supported.\n");
      #endif
      #ifdef _POSIX_POSIX_PII
         printf("   Protocol-independent interfaces are supported.\n");
      #else
         printf("   Protocol-independent interfaces are not supported.\n");
      #endif
      #ifdef _POSIX_PII_XTI
         printf("   XTI protocol-indep. interfaces are supported.\n");
      #else
         printf("   XTI protocol-indep. interfaces are not supported.\n");
      #endif
      #ifdef _POSIX_PII_SOCKET
         printf("   Socket protocol-indep. interfaces are supported.\n");
      #else
         printf("   Socket protocol-indep. interfaces are not supported.\n");
      #endif
      #ifdef _POSIX_PII_INTERNET
         printf("   Internet family of protocols is supported.\n");
      #else
         printf("   Internet family of protocols is not supported.\n");
      #endif
      #ifdef _POSIX_PII_INTERNET_STREAM
         printf("   Connection-mode Internet protocol is supported.\n");
      #else
         printf("   Connection-mode Internet protocol is not supported.\n");
      #endif
      #ifdef _POSIX_PII_INTERNET_DGRAM
         printf("   Connectionless Internet protocol is supported.\n");
      #else
         printf("   Connectionless Internet protocol is not supported.\n");
      #endif
      #ifdef _POSIX_PII_OSI
         printf("       ISO/OSI family of protocols is supported.\n");
      #else
         printf("       ISO/OSI family of protocols is not supported.\n");
      #endif
      #ifdef _POSIX_PII_OSI_COTS
         printf("   Connection-mode ISO/OSI service is supported.\n");
      #else
         printf("   Connection-mode ISO/OSI service is not supported.\n");
      #endif
      #ifdef _POSIX_PII_OSI_CLTS
         printf("   Connectionless ISO/OSI service is supported.\n");
      #else
         printf("   Connectionless ISO/OSI service is not supported.\n");
      #endif
      #ifdef _POSIX_POLL
         printf("   Implementation supports `poll' function.\n");
      #else
         printf("   poll function is not  supported.\n");
      #endif
      #ifdef _POSIX_SELECT
         printf("   Implementation supports `select' and `pselect'.\n");
      #else
         printf("   Implementation does not support `select' and `pselect'.\n");
      #endif
      #ifdef _XOPEN_REALTIME
         printf("   X/Open realtime support is available.\n");
      #else
         printf("   X/Open realtime support is not available..\n");
      #endif
      #ifdef _XOPEN_REALTIME_THREADS
         printf("   X/Open realtime thread support is available.\n");
      #else
         printf("   X/Open realtime thread support is not available.\n");
      #endif
      #ifdef _XOPEN_SHM
         printf("   XPG4.2 Shared memory interface is supported.\n");
      #else
         printf("   XPG4.2 Shared memory interface is not supported.\n");
      #endif
      #ifdef _XBS5_ILP32_OFF32
         printf("   32-bit int, long, pointer, and off_t types are supported.\n");
      #else
         printf("   32-bit int, long, pointer, and off_t types are not supported.\n");
      #endif
      #ifdef _XBS5_ILP32_OFFBIG
         printf("   32-bit int, long, and pointer and off_t with at least 64 bits is supported.\n");
      #else
         printf("   32-bit int, long, and pointer and off_t with at least 64 bits is not supported.\n");
      #endif
      #ifdef _XBS5_LP64_OFF64
         printf("   32-bit int, and 64-bit long, pointer, and off_t types are supported.\n");
      #else
         printf("   32-bit int, and 64-bit long, pointer, and off_t types are not supported.\n");
      #endif
      #ifdef _XBS5_LPBIG_OFFBIG
         printf("   32 bits int and long, pointer, and off_t with at least 64 bits are supported.\n");
      #else
         printf("   32 bits int and long, pointer, and off_t with at least 64 bits are not supported.\n");
      #endif


   #endif
   //Do POSIX.1 tests
   printf ("POSIX.1 Options Test Results:\n");
   #ifdef _POSIX_JOB_CONTROL
      printf("   POSIX.1 Job control is supported.\n");
   #else
      printf("   POSIX.1 Job control is not supported.\n");
   #endif
   #ifdef _POSIX_CHOWN_RESTRICTED
      printf("   POSIX.1 Chown restrictions are supported.\n");
   #else
      printf("   POSIX.1 Chown restrictions are not supported.\n");
   #endif
   #ifdef _POSIX_SAVED_IDS
      printf("   POSIX.1 Process saved IDs are supported.\n");
   #else
      printf("   POSIX.1 Process saved IDs are not supported.\n");
   #endif
   #ifdef _POSIX_NO_TRUNC
      printf("   POSIX.1 Long pathname errors are supported.\n");
   #else
      printf("   POSIX.1 Long pathname errors are not supported.\n");
   #endif
   #ifdef _POSIX_VDISABLE
      printf("   POSIX.1 Some terminal charactistics disabling is supported.\n");
   #else
      printf("   POSIX.1 Some terminal charactistics disabling is not supported.\n");
   #endif
   #ifdef NGROUPS_MAX
      printf("   POSIX.1 Supplementary group IDs is supported.\n");
   #else
      printf("   POSIX.1 Supplementary group IDs is not supported.\n");
   #endif

   //System Run time testing
   printf("\nSystem run time tests:\n");
   errno=0;
   val=sysconf(_SC_JOB_CONTROL);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_JOB_CONTROL\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.1 JOB CONTROL not Supported.\n");
   }
   else
   {
     printf("POSIX.1 JOB CONTROL Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_SAVED_IDS);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_SAVED_IDS\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.1 SAVED IDS not Supported.\n");
   }
   else
   {
     printf("POSIX.1 SAVED IDS Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_VERSION);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_VERSION\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.1 VERSION not Supported.\n");
   }
   else
   {
     printf("POSIX.1 VERSION Supported.\n");
   }
   val=sysconf(_SC_ARG_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_ARG_MAX\n");
   }
   else
   {
     printf("POSIX.1 ARG MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_CHILD_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_CHILD_MAX\n");
   }
   else
   {
     printf("POSIX.1 CHILD MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_CLK_TCK);
   if (val == -1)
   {
     printf("Bad option _SC_CLK_TCK\n");
   }
   else
   {
     printf("POSIX.1 CLK TCK Value=%ld.\n",val);
   }
   val=sysconf(_SC_NGROUPS_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_NGROUPS_MAX\n");
   }
   else
   {
     printf("POSIX.1 NGROUPS MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_OPEN_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_OPEN_MAX\n");
   }
   else
   {
     printf("POSIX.1 OPEN MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_STREAM_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_STREAM_MAX\n");
   }
   else
   {
     printf("POSIX.1 STREAM MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_TZNAME_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_TZNAME_MAX\n");
   }
   else
   {
     printf("POSIX.1 TZNAME MAX Value=%ld.\n",val);
   }
   errno=0;
   val=sysconf(_SC_ASYNCHRONOUS_IO);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_ASYNCHRONOUS_IO\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 ASYNCHRONOUS IO not Supported.\n");
   }
   else
   {
     printf("POSIX.4 ASYNCHRONOUS IO Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_MAPPED_FILES);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_MAPPED_FILES\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 MAPPED FILES not Supported.\n");
   }
   else
   {
     printf("POSIX.4 MAPPED FILES Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_MEMLOCK_RANGE);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_MEMLOCK_RANGE\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 MEMLOCK RANGE not Supported.\n");
   }
   else
   {
     printf("POSIX.4 MEMLOCK RANGE Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_MEMORY_PROTECTION);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_MEMORY_PROTECTION\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 MEMORY PROTECTION not Supported.\n");
   }
   else
   {
     printf("POSIX.4 MEMORY PROTECTION Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_MESSAGE_PASSING);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_MESSAGE_PASSING\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 MESSAGE PASSING not Supported.\n");
   }
   else
   {
     printf("POSIX.4 MESSAGE PASSING Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_PRIORITIZED_IO);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_PRIORITIZED_IO\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 PRIORITIZED IO not Supported.\n");
   }
   else
   {
     printf("POSIX.4 PRIORITIZED IO Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_PRIORITY_SCHEDULING);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_PRIORITY_SCHEDULING\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 PRIORITY SCHEDULING not Supported.\n");
   }
   else
   {
     printf("POSIX.4 PRIORITY SCHEDULING Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_REALTIME_SIGNALS);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_REALTIME_SIGNALS\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 REALTIME SIGNALS not Supported.\n");
   }
   else
   {
     printf("POSIX.4 REALTIME SIGNALS Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_SEMAPHORES);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_SEMAPHORES\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 SEMAPHORES not Supported.\n");
   }
   else
   {
     printf("POSIX.4 SEMAPHORES Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_FSYNC);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_FSYNC\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 FSYNC not Supported.\n");
   }
   else
   {
     printf("POSIX.4 FSYNC Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_SHARED_MEMORY_OBJECTS);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_SHARED_MEMORY_OBJECTS\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 SHARED_MEMORY_OBJECTS not Supported.\n");
   }
   else
   {
     printf("POSIX.4 SHARED_MEMORY_OBJECTS Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_SYNCHRONIZED_IO);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_SYNCHRONIZED_IO\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 SYNCHRONIZED IO not Supported.\n");
   }
   else
   {
     printf("POSIX.4 SYNCHRONIZED IO Supported.\n");
   }
   errno=0;
   val=sysconf(_SC_TIMERS);
   if ((val == -1) && (errno))
   {
     printf("Bad option _SC_TIMERS\n");
   }
   else if ((val == -1) && (!errno))
   {
     printf("POSIX.4 TIMERS not Supported.\n");
   }
   else
   {
     printf("POSIX.4 TIMERS Supported.\n");
   }
   val=sysconf(_SC_AIO_LISTIO_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_AIO_LISTIO_MAX\n");
   }
   else
   {
     printf("POSIX.4 AIO LISTIO MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_AIO_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_AIO_MAX\n");
   }
   else
   {
     printf("POSIX.4 AIO MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_AIO_PRIO_DELTA_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_AIO_PRIO_DELTA_MAX\n");
   }
   else
   {
     printf("POSIX.4 AIO PRIO DELTA MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_DELAYTIMER_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_DELAYTIMER_MAX\n");
   }
   else
   {
     printf("POSIX.4 DELAYTIMER MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_MQ_OPEN_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_MQ_OPEN_MAX\n");
   }
   else
   {
     printf("POSIX.4 MQ OPEN MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_MQ_PRIO_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_MQ_PRIO_MAX\n");
   }
   else
   {
     printf("POSIX.4 MQ PRIO MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_PAGESIZE);
   if (val == -1)
   {
     printf("Bad option _SC_PAGESIZE\n");
   }
   else
   {
     printf("POSIX.4 PAGESIZE Value=%ld.\n",val);
   }
   val=sysconf(_SC_RTSIG_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_RTSIG_MAX\n");
   }
   else
   {
     printf("POSIX.4 RTSIG MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_SEM_NSEMS_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_SEM_NSEMS_MAX\n");
   }
   else
   {
     printf("POSIX.4 SEM NSEMS MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_SEM_VALUE_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_SEM_VALUE_MAX\n");
   }
   else
   {
     printf("POSIX.4 SEM VALUE MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_SIGQUEUE_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_SIGQUEUE_MAX\n");
   }
   else
   {
     printf("POSIX.4 SIGQUEUE MAX Value=%ld.\n",val);
   }
   val=sysconf(_SC_TIMER_MAX);
   if (val == -1)
   {
     printf("Bad option _SC_TIMER_MAX\n");
   }
   else
   {
     printf("POSIX.4 TIMER MAX Value=%ld.\n",val);
   }

#endif

   return 0;
}
