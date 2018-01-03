^^^^^^^^^
Changelog
^^^^^^^^^

0.61.6 (2017-02-05)
-------------------
* add additional ubuntu releases to ecl_detect_distro
* cmake message bugfixes to always include type flag (removes warnings)

0.61.5 (2016-08-25)
-------------------
* avoid re-loading cotire for every project

0.61.4 (2016-03-20)
-------------------
* check, but just quietly avoid including cotire if version check fails.

0.61.3 (2016-02-23)
-------------------
* added corite cmake module for speeding up builds

0.61.2 (2016-01-09)
-------------------
* some comments about tricky c++11 hazards.

0.61.1 (2015-08-16)
-------------------
* ecl_detect_filesystem cmake macro added.

0.61.0 (2014-09-12)
-------------------
* convenience macros for cxx11 flag checks and settings.

0.60.1 (2014-01-29)
-------------------
* ecl_check_for->ecl_detect for consistency.
* use catkin exports for cmake modules, closes `#2 <https://github.com/stonier/ecl_tools/issues/2>`_
* Contributors: Daniel Stonier
