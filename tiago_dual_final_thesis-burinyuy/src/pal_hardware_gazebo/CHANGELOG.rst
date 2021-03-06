^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_hardware_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2021-08-26)
------------------
* Merge branch 'kangaroo_wbc' into 'ferrum-devel'
  Kangaroo wbc
  See merge request control/pal_hardware_gazebo!11
* remove unused pal_utils header
* Fix race condition when no controller is loaded yet
* Allow interface in effort and or in position
* better debug prints
* added saturation interface to enforce hard limit
* fixed the issue with joint handle for no softlimits
* added other potential fixes
* fixed the bug with the number of transmission actuators
* fixed the potential null pointer dereferencing
* added support for EffortJointInterface in the new transmission hw simulation
* added support for the Prismatic joints
* removed a shadowing typedef declaration
* remove curr_limit_cmd interface
* Added pal_hardware_gazebo/PalHardwareTransmissionGazebo plugin for custom robot sim interface
* Add README
* Contributors: Adria Roig, Sai Kishor Kothakota, Victor Lopez, victor

1.0.0 (2019-09-12)
------------------
* Fixed shadowed variables
* UUID error fix and Eigen version update
* added gazebo 9 and melodic changes
* Contributors: Jordan Palacios, Sai Kishor Kothakota

0.1.2 (2018-10-25)
------------------
* Merge branch 'migrate-to-statistics' into 'erbium-devel'
  Migrate to statistics
  See merge request control/pal_hardware_gazebo!6
* Migrate to statistics
* Merge branch 'check_version' into 'erbium-devel'
  added check gazebo version for compiling imu fix
  See merge request control/pal_hardware_gazebo!5
* fixe typo major -> minor
* fixed major minor issue
* added check gazebo version for compiling imu fix
* Merge branch 'fix_gazebo_8' into 'erbium-devel'
  fixed compilation issues
  See merge request control/pal_hardware_gazebo!4
* fixed compilation issues
* Contributors: Hilario Tome, Victor Lopez

0.1.1 (2018-04-06)
------------------
* Merge branch 'dynamic_introspection' into 'erbium-devel'
  added dynamic introspection
  See merge request control/pal_hardware_gazebo!3
* fixed imu world reference
* added dynamic introspection
* Contributors: Hilario Tome

0.1.0 (2018-01-17)
------------------
* formating
* Added gazebo7 suppot
* Contributors: Hilario Tome, Hillario Tome

0.0.8 (2016-10-21)
------------------
* add control_toolboxes in find_package
* Contributors: Jordi Pages

0.0.7 (2016-10-14)
------------------

0.0.6 (2016-10-12)
------------------
* gazebo include dirs
* Removed duplicated gazebo plugin
* Added gazebo depend
* Merge branch 'dubnium-devel' of gitlab:control/pal_hardware_gazebo into dubnium-devel
* Added pal_hardware_interfaces depend
* Removed hardcoded imu
* Contributors: Hilario Tome

0.0.5 (2016-06-30)
------------------
* Merge branch 'dubnium-devel' of gitlab:control/pal_hardware_gazebo into dubnium-devel
* Bug fix for when the ft sensors is the first link
* Contributors: Hilario Tome

0.0.4 (2016-05-06)
------------------
* Added missing dependencies
* Added catkin depends so api/abi checking works
* Contributors: Sam Pfeiffer

0.0.3 (2016-05-06)
------------------
* Add explicitly eigen dependency
* Pulled necessary functions from pal_robot_tools to here to remove dependecy tree
* Removed unused dependencies
* Contributors: Sam Pfeiffer

0.0.2 (2016-04-14)
------------------
* Added missing IMU parsing and clean up
* Contributors: Hilario Tome

0.0.1 (2016-03-31)
------------------
* Sort of made sense of gazebo ft feedback
* Added correct whrench tranformation
* Added imu ft parsing
* Initial commit
* Contributors: Hilario Tome
