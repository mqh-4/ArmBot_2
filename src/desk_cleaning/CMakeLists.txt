cmake_minimum_required(VERSION 3.0.2)
project(desk_cleaning)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)

add_action_files(FILES
  CleanDesk.action
)

add_service_files(FILES
  SetSimGrasp.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  rospy
  sensor_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/desk_perception_node.py
  scripts/desk_visualizer.py
  scripts/eye_to_hand_calibration.py
  scripts/eye_to_hand_tf_publisher.py
  scripts/calibration_arm_teleop.py
  scripts/desk_cleaning_executor.py
  scripts/voice_task_planner.py
  scripts/hardware_asr_bridge.py
  scripts/usb_whisper_asr_node.py
  scripts/mock_perception_node.py
  scripts/tts_feedback.py
  scripts/sim_servo_bridge.py
  scripts/sim_truth_perception_node.py
  scripts/sim_grasp_manager.py
  scripts/sim_base_anchor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
