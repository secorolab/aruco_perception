cmake_minimum_required(VERSION 3.16)
project(aruco_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LocateObjects.action"
  DEPENDENCIES vision_msgs
)

# rosidl_generate_interfaces already installs a python package named ${PROJECT_NAME};
# merge the node modules into it instead of installing a second package of the same name
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__init__.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  scripts/world_pose_node
  scripts/detect_objects_node
  scripts/locate_objects_server
  scripts/mock_locate_server
  scripts/object_pose_node
  scripts/mock_object_pose_publisher
  scripts/handle_pose_node
  scripts/gen_sim_markers
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
