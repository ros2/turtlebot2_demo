=========
Changelog
=========

0.6.2 (2016-01-09)
------------------
* enable the cx11 compiler

0.6.0 (2014-08-04)
------------------
* Redevelopment of kobuki dock drive algorithm.
* Contributors: Jihoon Lee

0.5.3 (2013-09-06)
------------------

0.5.2 (2013-08-31)
------------------

0.5.1 (2013-08-30)
------------------

0.5.0 (2013-08-29)
------------------
* Added extra url info on all packages.
* adds params for kobuki_auto_docking launchers.
* Changelogs at package level.
* 32 bit alignment error. Fast fix: just remove the
  eigen-inheritor attribute, as it's not really needed. It was already
  commented in a previous commit; here I just cleanup and provide a
  description of the fix.
  But the fact is that something is wrong on ecl. We keep track on.
* Fixed Eigenlib alignment error on 32 bit architectures.

0.4.0 (2013-08-09)
------------------
* Add minimum speed parameters: with heavy payloads, at very low speeds, the robot can get stuck easily.
* Remove motors enabling/disabling, as it can be confusing and it's not particularly useful.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
