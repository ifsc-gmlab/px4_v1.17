cmake_minimum_required(VERSION 3.0.2)
project(flying_car_test)
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs geometry_msgs mavros_msgs rosgraph_msgs rospy std_msgs std_srvs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/experiment_controller.py
  scripts/experiment_monitor.py
  scripts/safety_guard.py
  scripts/validate_run.py
  scripts/generate_report.py
  scripts/run_trials.py
  scripts/stage_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
