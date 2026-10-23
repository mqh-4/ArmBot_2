cmake_minimum_required(VERSION 3.0.2)
project(AI_pkg)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs std_srvs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy std_msgs std_srvs)

catkin_install_python(PROGRAMS
  scripts/mcp_server_node.py
  scripts/llm_agent_node.py
  scripts/record_motion_bag.py
  scripts/voice_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING AND PYTHON_VERSION_STRING VERSION_GREATER_EQUAL "3.10")
  catkin_add_nosetests(test)
endif()
