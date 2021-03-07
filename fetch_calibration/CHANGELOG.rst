^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2021-03-06)
------------------
* Initial noetic release
* Add velocity-factor in calibrate_robot (`#133 <https://github.com/fetchrobotics/fetch_ros/issues/133>`_)
  Limit velocity factor so that it can only be used to slow down calibration.
* Updates maintainers
* Contributors: Alex Moriarty, Eric Relson, Russell Toris, Shingo Kitagawa

0.8.2 (2019-08-06)
------------------

0.8.1 (2019-04-04)
------------------
* sync cmake_minimum_required: 2.8.12
* Contributors: Alexander Moriarty

0.8.0 (2019-02-13)
------------------
* [package.xml] REP-140 package format 2 (`#104 <https://github.com/fetchrobotics/fetch_ros/issues/104>`_)
  closes `#83 <https://github.com/fetchrobotics/fetch_ros/issues/83>`_
* Merge pull request `#100 <https://github.com/fetchrobotics/fetch_ros/issues/100>`_ from aparker-fetch/distro-agnostic-calibration
  removed hardcoded ROS distro from calibrate_robot
* removed hardcoded ROS distro from calibrate_robot
* [Fetch Calibration] fix syntax error (`#96 <https://github.com/fetchrobotics/fetch_ros/issues/96>`_)
  Noticed here https://index.ros.org/stats/errors/
  Failed to parse launchfile launch/capture_manual.launch:
  Missing end tag for 'rosparam' (got "launch")
  Line: 16 Position: 668 Last 80 unconsumed characters:
* [Docs] Add URL tags to package for wiki.ros.org (`#90 <https://github.com/fetchrobotics/fetch_ros/issues/90>`_)
  The <url> tag is required to automatically fill in at least some info
  on the wiki pages. The extra tags will create links to our docs.
* Merge pull request `#81 <https://github.com/fetchrobotics/fetch_ros/issues/81>`_ from moriarty/tf2-nav-melodic-devel
  [fetch_depth_layer][tf2] fixes for upstream navigation
  Changes for compatibility with `ros-planning/navigation#755 <https://github.com/ros-planning/navigation/issues/755>`_
* (fetch_calibration) updates for new finders
* Contributors: Alex Moriarty, Andrew Parker, Eric Relson, Michael Ferguson

0.7.14 (2018-07-10)
-------------------
* updates ownership
* Contributors: Russell Toris

0.7.13 (2017-11-02)
-------------------

0.7.12 (2017-08-02)
-------------------

0.7.11 (2017-07-31)
-------------------

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
* added support for ground plane calibration
* Contributors: Niharika Arora

0.6.2 (2015-07-30)
------------------
* fix parameter delete
* Contributors: Michael Ferguson

0.6.1 (2015-07-03)
------------------
* delete old parameters before reloading
* use dated temp files to avoid permissions issues, fixes `#9 <https://github.com/fetchrobotics/fetch_ros/issues/9>`_
* specify camera/chain names
* add checkboard based calibration config
* Contributors: Michael Ferguson

0.6.0 (2015-06-23)
------------------
* update capture for new multi_sensor branch of calibration
* Contributors: Michael Ferguson

0.5.14 (2015-06-19)
-------------------

0.5.13 (2015-06-13)
-------------------

0.5.12 (2015-06-12)
-------------------

0.5.11 (2015-06-10)
-------------------
* Update calibration poses bag file
* Contributors: varunns

0.5.10 (2015-06-07)
-------------------

0.5.9 (2015-06-07)
------------------
* fix head camera parameter names
* Contributors: Michael Ferguson

0.5.8 (2015-06-07)
------------------
* add script for disabling auto exposure/whitebalance of head camera
* update calibration config
* Contributors: Michael Ferguson

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
* add launch file argument for velocity_factor
* Contributors: Michael Ferguson

0.5.2 (2015-04-19)
------------------
* update calibration configuration
* Contributors: Michael Ferguson

0.5.1 (2015-04-09)
------------------

0.5.0 (2015-04-04)
------------------
* First public release
* Contributors: Michael Ferguson
