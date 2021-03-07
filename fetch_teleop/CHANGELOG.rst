^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2021-03-06)
------------------
* Initial noetic release
* Fix tuck arm moveit check [OPEN-48] (`#150 <https://github.com/fetchrobotics/fetch_ros/issues/150>`_)
* Tuck arm script: Add ground collision for melodic (`#130 <https://github.com/fetchrobotics/fetch_ros/issues/130>`_)
and (`#132 <https://github.com/fetchrobotics/fetch_ros/issues/132>`_)
  * add ground as collision object
  * use mesh for tuck arm collision
* Updates maintainers
* Contributors: Alex Moriarty, Jeff Wilson, Russell Toris, Shingo Kitagawa

0.8.2 (2019-08-06)
------------------
* Merge pull request `#118 <https://github.com/fetchrobotics/fetch_ros/issues/118>`_ from 708yamaguchi/add-teleop-rosparam-melodic
  Select teleop part by rosparam for melodic
* select teleop part by rosparam
* Contributors: Carl Saldanha, Naoya Yamaguchi

0.8.1 (2019-04-04)
------------------
* sync cmake_minimum_required: 2.8.12
* Contributors: Alexander Moriarty

0.8.0 (2019-02-13)
------------------
* [package.xml] REP-140 package format 2 (`#104 <https://github.com/fetchrobotics/fetch_ros/issues/104>`_)
  closes `#83 <https://github.com/fetchrobotics/fetch_ros/issues/83>`_
* [Docs] Add URL tags to package for wiki.ros.org (`#90 <https://github.com/fetchrobotics/fetch_ros/issues/90>`_)
  The <url> tag is required to automatically fill in at least some info
  on the wiki pages. The extra tags will create links to our docs.
* [teleop] Require primary dead-man for arm motion (`#92 <https://github.com/fetchrobotics/fetch_ros/issues/92>`_)
  * Require primary dead-man for arm teleop, along with with the angular or linear selector button.
* Fixup to add missing variable definition.
* Also require primary deadman for arm teleop
* Contributors: Alex Moriarty, Eric Relson

0.7.14 (2018-07-10)
-------------------
* updates ownership
* Merge pull request `#68 <https://github.com/fetchrobotics/fetch_ros/issues/68>`_ from BillWSY/indigo-devel
  Fixes an issue that tuck_arm.py spawns new roscore.
* Fixes an issue that tuck_arm.py spawns new roscore.
  tuck_arm.py calls roslaunch without --wait flag, which may spawn new ROS
  master in a racing condition.
* Contributors: Michael Ferguson, Russell Toris, Shengye Wang

0.7.13 (2017-11-02)
-------------------

0.7.12 (2017-08-02)
-------------------

0.7.11 (2017-07-31)
-------------------
* simpified with easier controls
* add joystick teleop for arm control
* Contributors: Hanjun Song, Michael Ferguson

0.7.10 (2016-10-27)
-------------------

0.7.9 (2016-07-26)
------------------

0.7.8 (2016-07-18)
------------------

0.7.7 (2016-06-20)
------------------

0.7.6 (2016-05-26)
------------------
* fix attribute error when moveit does not return
* Contributors: Michael Ferguson

0.7.5 (2016-05-08)
------------------
* Control gripper via keyboard
* Contributors: Kentaro Wada

0.7.4 (2016-03-16)
------------------

0.7.3 (2016-03-05)
------------------

0.7.2 (2016-02-24)
------------------

0.7.1 (2016-01-20)
------------------

0.7.0 (2015-09-29)
------------------
* Require deadman to be held while tucking
* add missing moveit_python depend
* Contributors: Alex Henning, Michael Ferguson

0.6.2 (2015-07-30)
------------------

0.6.1 (2015-07-03)
------------------
* make pan/tilt acceleration parameterized
* update head tilt joint limits in teleop
* Contributors: Michael Ferguson

0.6.0 (2015-06-23)
------------------
* install tuck_arm.py
* add a keepout zone for extra margin around base
* add ability to run tuck arm script without joystick
* Contributors: Michael Ferguson

0.5.14 (2015-06-19)
-------------------
* add script to tuck arm
* Contributors: Michael Ferguson

0.5.13 (2015-06-13)
-------------------

0.5.12 (2015-06-12)
-------------------

0.5.11 (2015-06-10)
-------------------

0.5.10 (2015-06-07)
-------------------
* fix random glitches due to having two joint states publishers
* Contributors: Michael Ferguson

0.5.9 (2015-06-07)
------------------

0.5.8 (2015-06-07)
------------------

0.5.7 (2015-06-05)
------------------

0.5.6 (2015-06-04)
------------------

0.5.5 (2015-06-03)
------------------

0.5.4 (2015-05-09)
------------------
* repository cleanup

0.5.3 (2015-05-03)
------------------
* set a slower reverse speed for teleop
* Contributors: Michael Ferguson

0.5.2 (2015-04-19)
------------------
* prevent teleop command from winding up ahead of actual velocity
* use -Wall, publish should return void
* stop supressed components
* Contributors: Michael Ferguson

0.5.1 (2015-04-09)
------------------

0.5.0 (2015-04-04)
------------------

0.4.2 (2015-03-23)
------------------

0.4.1 (2015-03-23)
------------------

0.4.0 (2015-03-22)
------------------
* initial release
* Contributors: Michael Ferguson
