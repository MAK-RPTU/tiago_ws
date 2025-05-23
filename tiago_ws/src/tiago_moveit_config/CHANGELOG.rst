^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.2 (2025-02-05)
------------------
* Set robot_description_timeout to 60 seconds
* Contributors: Noel Jimenez

3.1.1 (2024-11-04)
------------------
* Merge branch 'tpe/fix_base_type_arg' into 'humble-devel'
  Add base_type arg to move_group
  See merge request robots/tiago_moveit_config!88
* Add base_type arg to move_group
* Contributors: thomas.peyrucain, thomaspeyrucain

3.1.0 (2024-09-18)
------------------
* remove ft-sensor in hw suffix and files
* Contributors: David ter Kuile

3.0.18 (2024-07-25)
-------------------
* Change parameter to arm_type
* Contributors: thomas.peyrucain

3.0.17 (2024-07-09)
-------------------
* Add warning for pal_module_cmake not found
* Contributors: Noel Jimenez

3.0.16 (2024-06-26)
-------------------
* Merge branch 'dtk/move-robot-args' into 'humble-devel'
  Change import for launch args
  See merge request robots/tiago_moveit_config!84
* Change import for launch args
* Contributors: David ter Kuile, davidterkuile

3.0.15 (2024-06-10)
-------------------
* Merge branch 'feat/motions' into 'humble-devel'
  add custom file for posible costumers
  See merge request robots/tiago_moveit_config!83
* new disable collision for robotiq
* add disable collision
* add custom file for posible costumers
* Contributors: Aina, davidterkuile

3.0.14 (2024-05-13)
-------------------
* Merge branch 'omm/feat/imarkers' into 'humble-devel'
  Restored proper rviz launch file with interactive markers
  See merge request robots/tiago_moveit_config!82
* Restored proper rviz launch file with interactive markers
* Contributors: davidterkuile, oscarmartinez

3.0.13 (2024-05-09)
-------------------
* Merge branch 'feat/auto-generated_srdf_files' into 'humble-devel'
  auto generated srdf files
  See merge request robots/tiago_moveit_config!81
* include files
* create srdf on the go
* migrate update.sh
* restructure srdf files
* add disable collision files & end_effectors files
* Merge branch 'omm/fix/launch_standarization' into 'humble-devel'
  Launch files moved to TIAGo family standard
  See merge request robots/tiago_moveit_config!79
* Removing deprecated fake controllers
* Launch files moved to TIAGo family standard
* Merge branch 'dtk/fix/missing-dependency' into 'humble-devel'
  Fix missing dependency
  See merge request robots/tiago_moveit_config!78
* Add missing moveit-ros-perception dependency
* Contributors: Aina Irisarri, David ter Kuile, Noel Jimenez, Oscar, davidterkuile

3.0.12 (2024-03-06)
-------------------
* Enable log colors for move_group and rviz nodes
* Revert "Install the moveit setup assistant to avoid some warnings"
  This reverts commit 7d51b736a0d22bd12bad43f9322f1a52c4532d46.
* Contributors: Noel Jimenez

3.0.11 (2024-03-04)
-------------------
* Install the moveit setup assistant to avoid some warnings
* Contributors: Noel Jimenez

3.0.10 (2024-02-29)
-------------------
* Read robot_description from the topic
* Contributors: Noel Jimenez

3.0.9 (2024-01-31)
------------------
* delete arm controllers when no-arm
* change config files name when no end_effector & adding robotiq config files
* Contributors: Aina Irisarri

3.0.8 (2024-01-12)
------------------
* Publish robot_description_semantic
* Contributors: Noel Jimenez

3.0.7 (2023-12-18)
------------------
* Add if statement for moveit sensor manager
* Newline at the end of the file
* Add moveit sensor params
* Contributors: David ter Kuile, Noel Jimenez

3.0.6 (2023-11-14)
------------------
* Update website
* Contributors: Noel Jimenez

3.0.5 (2023-11-13)
------------------
* Remove use_sim_time from module
* Set use_sim_time false as default
* Contributors: Noel Jimenez

3.0.4 (2023-11-07)
------------------
* Specify arguments for move_group module
* Contributors: Noel Jimenez

3.0.3 (2023-09-21)
------------------
* Merge branch 'add_modules' into 'humble-devel'
  Adding move_group module
  See merge request robots/tiago_moveit_config!61
* Adding move_group module
* Merge branch 'remove_pal_flags_dependency' into 'humble-devel'
  Remove pal flags dependency
  See merge request robots/tiago_moveit_config!60
* Remove pal flags dependency
* Contributors: Jordan Palacios, Noel Jimenez

3.0.2 (2023-06-14)
------------------
* config files regeneration
* update get_tiago_hw_suffix method usage
* Contributors: Noel Jimenez

3.0.1 (2023-02-08)
------------------
* Merge branch 'fix_move_group' into 'humble-devel'
  Add missing pilz_cartesian_limits for moveit config
  See merge request robots/tiago_moveit_config!50
* add missing pilz_cartesian_limits for moveit config
* Contributors: Jordan Palacios, Noel Jimenez

3.0.0 (2022-11-29)
------------------
* Merge pull request #11 from AndyZe/andyz/cm_update
  [Humble] Update the ControllerManger name
* Update in the .em file as well
* Update the Controller Manager name
* Merge branch 'add_missing_dependencies' into 'humble-devel'
  Add missing dependencies
  See merge request robots/tiago_moveit_config!46
* add missing dependencies
* Merge branch 'update_rviz_cfg' into 'humble-devel'
  Update rviz config
  See merge request robots/tiago_moveit_config!45
* update rviz cfg
* Merge branch 'cleanup' into 'humble-devel'
  update pkg deps
  See merge request robots/tiago_moveit_config!44
* update pkg deps
* Merge branch 'linters' into 'humble-devel'
  Linters
  See merge request robots/tiago_moveit_config!43
* linters
* copyright
* CONTRIBUTING.md
* add linters
* Merge branch 'update_launchers' into 'humble-devel'
  Update moveit launchers refactor
  See merge request robots/tiago_moveit_config!42
* update config and use MoveItConfigsBuilder for launchers
* regenerate controller yaml files
* rm name to avoid duplicated node
* update moveit launchers refactor
* Merge branch 'refactor_ld' into 'humble-devel'
  Refactor LaunchDescription population
  See merge request robots/tiago_moveit_config!41
* refactor LaunchDescription population
* Merge branch 'license' into 'humble-devel'
  Add Apache License
  See merge request robots/tiago_moveit_config!40
* add LICENSE
* Merge branch 'cleanup' into 'humble-devel'
  Cleanup
  See merge request robots/tiago_moveit_config!39
* rm ros1 launchers
* Merge branch 'update_maintainers' into 'humble-devel'
  update maintainers
  See merge request robots/tiago_moveit_config!38
* update maintainers
* Merge branch 'separate_rviz_and_move_group' into 'foxy-devel'
  Separate rviz from move group launcher
  See merge request robots/tiago_moveit_config!33
* separate rviz from move_group launcher
* Update rviz config
* Add camera_model to description generator
* Style and cleanup
* Use rviz in this repo
* Update move_group for all tiago configurations
* Fix controllers and srdf generation
* Regenerate srdf and controllers for ROS2
* Initial hard coded ROS2 version
* Contributors: AndyZe, Jordan Palacios, Noel Jimenez, Noel Jimenez Garcia, Sai Kishor Kothakota, Victor Lopez

1.1.1 (2021-05-06)
------------------

1.1.0 (2021-05-06)
------------------
* Merge branch 'robotiq_gripper' into 'erbium-devel'
  Robotiq gripper
  See merge request robots/tiago_moveit_config!22
* update the SRDF with the missing FT links
* Update the SRDF configuration
* update the robotiq end effector naming
* initial commit of robotiq 85 and 140 moveit config of TIAGo
* Add README and update setup assistant xacro file name
* Contributors: Sai Kishor Kothakota, Victor Lopez, saikishor

1.0.6 (2020-10-01)
------------------
* Merge branch 'hey5_marker' into 'erbium-devel'
  Hey5 marker
  See merge request robots/tiago_moveit_config!21
* Hey5 marker
* Contributors: Adria Roig, victor

1.0.5 (2020-06-09)
------------------
* Add arm_5 wrist ignore collisions
* Contributors: Victor Lopez

1.0.4 (2020-04-21)
------------------
* Merge branch 'custom-ee' into 'erbium-devel'
  Allow using custom end-effector
  See merge request robots/tiago_moveit_config!19
* Allow using custom end-effector
* Contributors: davidfernandez, victor

1.0.3 (2020-02-06)
------------------
* Merge branch 'move_group_capability' into 'erbium-devel'
  send capabilities through args
  See merge request robots/tiago_moveit_config!18
* send capabilities through args
* Contributors: Victor Lopez, YueErro

1.0.2 (2019-08-22)
------------------
* Add missing multi argument
* Decrease segment size for validation
* Contributors: Victor Lopez

1.0.1 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Added autogenerated srdf
  See merge request robots/tiago_moveit_config!16
* Restore old camera parameter
* Refactor controllers files
* Refactor joint limits and srdf
* Added autogenerated srdf
* Contributors: Victor Lopez

1.0.0 (2018-12-19)
------------------

0.0.22 (2018-07-30)
-------------------
* Merge branch 'fix-simulation-warnings' into 'cobalt-devel'
  fix deprecated namespace
  See merge request robots/tiago_moveit_config!15
* fix deprecated namespace
* fix demo mode by adding missing argument
  You hacked multi-robot support into a generated moveit configuration
  but didn't test "roslaunch tiago_moveit_config demo.launch".
  I agree that gazebo support is better than the demo mode, but
  it can be very useful to test MoveIt-based code without controlling.
* Contributors: Jordi Pages, Victor Lopez, v4hn

0.0.21 (2018-03-28)
-------------------
* Merge branch 'disable-sonar-collision' into 'cobalt-devel'
  Disable sonar collision with base_link
  See merge request robots/tiago_moveit_config!14
* Disable sonar collision with base_link
* Contributors: Victor Lopez, davidfernandez

0.0.20 (2018-03-26)
-------------------
* Merge branch 'recover-chessboard-tiago' into 'cobalt-devel'
  Disable collision between arm 7 and chessboard
  See merge request robots/tiago_moveit_config!13
* Disable collision between arm 7 and chessboard
* Contributors: Jordi Pages, Victor Lopez

0.0.19 (2018-01-24)
-------------------
* add config files for schunk and some renamings
* Contributors: Jordi Pages

0.0.18 (2017-11-03)
-------------------
* Change the topic and the max_range for the octomap parameters
* Contributors: AleDF, Jordi Pages

0.0.17 (2017-05-16)
-------------------
* Merge branch 'octomap_track_ik' into 'cobalt-devel'
  merge_problems_with david
  See merge request !11
* merge_problems_with david
* Merge branch 'iron-configuration' into 'cobalt-devel'
  Add configuration for Tiago Iron
  See merge request !10
* Merge branch 'octomap_track_ik' into 'cobalt-devel'
  octomap & track ik solver for MoveIt!
  See merge request !9
* Add configuration for Tiago Iron
* octomap & track ik solver for MoveIt!
* Contributors: AleDF, Jordi Pages, davidfernandez

0.0.16 (2016-10-21)
-------------------
* fix maintainer
* add argument for steel and titanium versions
* add missing xml formatting
* add specific controllers for steel and titanium
* disable collision arm_5_link-gripper_link
* disable collision arm_6_link-wrist_ft_link
* add missing joints
* use soft links for steel and titanium srdf files
* disable collisions arm_5_link-gripper_link
* Contributors: Jordi Pages

0.0.15 (2016-07-08)
-------------------
* Merge branch 'add-titanium-collisions-with-ft' into 'cobalt-devel'
  add missing potential collisions with ft sensor frames
  See merge request !5
* add collisions with ft sensor
* Merge branch 'tiago_configs' into 'cobalt-devel'
  Added the 4 possible configurations of tiago_moveit_config
  See merge request !4
* Added the 4 possible configurations of tiago_moveit_config
* Contributors: Jordi Pages, Sam Pfeiffer, Victor Lopez

0.0.14 (2016-06-13)
-------------------
* Added necessary dependence to run moveit with a simulated or real robot
* Add disable collisions for force torque sensor
* Contributors: Sam Pfeiffer

0.0.13 (2016-06-01)
-------------------
* Added controllers for hand and gripper
* Contributors: Sam Pfeiffer

0.0.12 (2016-04-04)
-------------------
* Increase max speed of torso
* Contributors: Sam Pfeiffer

0.0.11 (2016-04-04)
-------------------
* Missing hand_palm_link in collision disables
* Contributors: Sam Pfeiffer

0.0.10 (2016-04-04)
-------------------
* Add disables in between hand finger links
  Without this, the robot will refuse to plan with closed hand
* Contributors: Sam Pfeiffer

0.0.9 (2016-03-31)
------------------
* Add disable collisions
  Using the generator.
  From:
  1300 / 2145 pairs disabled in tiago_titanium (845 enabled)
  To:
  2268 / 3096 pairs disabled in tiago_titanium (828 enabled)
* Add disable collisions
  Generated using https://gist.github.com/awesomebytes/18fe75b808c4c644bd3d a script that runs the urdf tree for adjacent links and checks for links without collision mesh to also disable the collision computation between them.
  From:
  (Generating matrix with max sampling density)
  329 / 465 pairs disabled in tiago_steel (136 enabled)
  To:
  754 / 873 pairs disabled in tiago_steel (119 enabled)
* Contributors: Sam Pfeiffer

0.0.8 (2016-03-18)
------------------
* Added impossible collision disabling between torso_fixed_column_link and arm_2_link
* Contributors: Sam Pfeiffer

0.0.7 (2016-03-18)
------------------
* Passing change to titanium too about torso_fixed_column_link collision with arm1 disabling
* Added another currently happening collision exception between torso_fixed_column_link and arm_1_link
* Contributors: Sam Pfeiffer

0.0.6 (2016-03-18)
------------------
* Add hand passive joints as passive
* added clear octomap and removed exceptions on collisions of arm wit hhead
* Contributors: Sam Pfeiffer

0.0.5 (2016-03-10)
------------------
* Refs #11489. Discard collisions between torsolinks
* Fix collisions with column
* Remove elements of prototype mobilebase
* Disable collision hand safety box <-> wrist mesh
* Add arm group + disable more internal hand collisions
* Contributors: Bence Magyar, jordi.pages@pal-robotics.com

0.0.4 (2015-05-20)
------------------
* Add hand_safety_box to the game!
* Disable more collisions between hand links
* Contributors: Bence Magyar

0.0.3 (2015-04-14)
------------------
* Fix gripper parts
* Add torso controller
* Separate configuration files for titanium and steel, launch files parametrized
* Contributors: Bence Magyar

0.0.2 (2015-01-20)
------------------
* Remove tiago_description dependency
* Contributors: Bence Magyar

0.0.1 (2015-01-20)
------------------
* Added configuration with arm controllers
* Initial version of tiago_moveit_config (no hand)
* Contributors: Sammy Pfeiffer
