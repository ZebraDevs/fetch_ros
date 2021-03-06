^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.1 (2021-03-05)
------------------

0.9.0 (2021-02-28)
------------------
* Initial noetic release
* Updates maintainers
* Contributors: Eric Relson

0.8.2 (2019-08-06)
------------------
* Fixed chomp configuration for porting to moveit melodic-devel (`#124 <https://github.com/fetchrobotics/fetch_ros/issues/124>`_)
* Merge pull request `#122 <https://github.com/fetchrobotics/fetch_ros/issues/122>`_ from dekent/melodic-devel
  Much more collision checking = much safer robot with minimal planning time increases
* Merge pull request `#116 <https://github.com/fetchrobotics/fetch_ros/issues/116>`_ from umhan35/moveit-costomap
  [moveit_config] allow moveit octomap params to be overridden
* add default moveit_octomap_sensor_params_file to move_group.launch
* allow moveit octomap params to be overridden
* Contributors: Carl Saldanha, David Kent, Yuma Hijioka, Zhao Han

0.8.1 (2019-04-04)
------------------
* sync cmake_minimum_required: 2.8.12
* Merge pull request `#107 <https://github.com/fetchrobotics/fetch_ros/issues/107>`_ from moriarty/update-ikfast-plugin
  [IKFast Plugin] Regenerate fetch_ikfast_plugin
  Fixes `#103 <https://github.com/fetchrobotics/fetch_ros/issues/103>`_
* Contributors: Alexander Moriarty, Carl Saldanha

0.8.0 (2019-02-13)
------------------
* [package.xml] REP-140 package format 2 (`#104 <https://github.com/fetchrobotics/fetch_ros/issues/104>`_)
  closes `#83 <https://github.com/fetchrobotics/fetch_ros/issues/83>`_
* [MoveIt][Melodic]: MoveGroupExecuteService -> MoveGroupExecuteTrajectoryAction (`#94 <https://github.com/fetchrobotics/fetch_ros/issues/94>`_)
* [Docs] Add URL tags to package for wiki.ros.org (`#90 <https://github.com/fetchrobotics/fetch_ros/issues/90>`_)
  The <url> tag is required to automatically fill in at least some info
  on the wiki pages. The extra tags will create links to our docs.
* Contributors: Alex Moriarty, ivandariojr

0.7.14 (2018-07-10)
-------------------
* updates ownership
* Contributors: Russell Toris

0.7.13 (2017-11-02)
-------------------
* add apply_planning_scene
* [moveit_config] Add CHOMP planner demo launch.
  Enabling `CHOMP` on RViz by following the [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/chomp_interface_tutorial.html) (which currently included false information. I'll open a PR right after this).
  Tested on Xenial-Kinetic
  This commit is not necessarily intended to merged in. But would be nice to be kept for the entire MoveIt! community as a reference if possible. At least this will remain on the fork of @130s.
  *This PR is sponsored by [PlusOne Robotics](http://plusonerobotics.com) as part of [World MoveIt! Day 2017 hackathon](http://moveit.ros.org/events/world-moveit-day-2017)*
* Contributors: Isaac I.Y. Saito, Shingo Kitagawa

0.7.12 (2017-08-02)
-------------------
* add dependency for moveit
* Contributors: Michael Ferguson

0.7.11 (2017-07-31)
-------------------

0.7.10 (2016-10-27)
-------------------
* update package.xmls, add depend on fetch_ikfast_plugin
* load the ikfast kinematics plugin
* Contributors: Di Sun, Michael Ferguson

0.7.9 (2016-07-26)
------------------

0.7.8 (2016-07-18)
------------------

0.7.7 (2016-06-20)
------------------

0.7.6 (2016-05-26)
------------------

0.7.5 (2016-05-08)
------------------

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
* fix dependency issue with run/test duplication
* add missing moveit_python depend
* fix name of gripper fingers in fake controllers
* Contributors: Michael Ferguson

0.6.2 (2015-07-30)
------------------

0.6.1 (2015-07-03)
------------------
* add (optional) octomap configuration
* Contributors: Michael Ferguson

0.6.0 (2015-06-23)
------------------

0.5.14 (2015-06-19)
-------------------

0.5.13 (2015-06-13)
-------------------

0.5.12 (2015-06-12)
-------------------

0.5.11 (2015-06-10)
-------------------

0.5.10 (2015-06-07)
-------------------

0.5.9 (2015-06-07)
------------------

0.5.8 (2015-06-07)
------------------

0.5.7 (2015-06-05)
------------------
* bump joint limits used for moveit
* Contributors: Michael Ferguson

0.5.6 (2015-06-04)
------------------

0.5.5 (2015-06-03)
------------------

0.5.4 (2015-05-09)
------------------
* repository cleanup

0.5.3 (2015-05-03)
------------------

0.5.2 (2015-04-19)
------------------

0.5.1 (2015-04-09)
------------------

0.5.0 (2015-04-04)
------------------
* First public release
* Contributors: Michael Ferguson
