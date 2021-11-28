execute_process(COMMAND "/mnt/c/final_project/106a-project/a1_soccer_vision/build/ros_numpy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/mnt/c/final_project/106a-project/a1_soccer_vision/build/ros_numpy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
