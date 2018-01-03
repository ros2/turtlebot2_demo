^^^^^^^^^
Changelog
^^^^^^^^^

0.61.12 (2016-05-03)
--------------------
* apple baud rate 460800

0.61.1 (2015-07-22)
-------------------
* catch the error code as a hint for when fcntl calls fail on serial devices
* additional baud rate enums for serial devices covering a wider range
* deprecated a couple of serial device baud rates that are no longer valid

0.61.0 (2014-09-12)
-------------------
* allow immediate socket reuse in the socket client/server
* define an undefined baud rate for apple

0.60.8 (2014-02-10)
-------------------
* control new posix.1-2008 SIGPIPE signals, `#34 <https://github.com/stonier/ecl_core/issues/34>`_
* Contributors: Daniel Stonier

0.60.7 (2014-02-03)
-------------------

0.60.6 (2014-01-29)
-------------------
