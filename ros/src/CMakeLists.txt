cmake_minimum_required(VERSION 2.8.3)

project(pacecat)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  message_generation
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -Wfatal-errors")

add_service_files(
   FILES
   Control.srv
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package(
CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(PACECAT_SDK_PATH "./sdk/")
FILE(GLOB PACECAT_SDK_SRC
   "${PACECAT_SDK_PATH}/src/*.cpp",
   "${PACECAT_SDK_PATH}/src/*.c"
)

add_executable(${PROJECT_NAME}_node  ./src/node.cpp ${PACECAT_SDK_SRC})
add_executable(${PROJECT_NAME}_client ./src/client.cpp)
add_executable(${PROJECT_NAME}_playback_node ./src/playback_node.cpp ${PACECAT_SDK_SRC})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg  pacecat_generate_messages_cpp   ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${PROJECT_NAME}_gencfg pacecat_generate_messages_cpp   ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_playback_node ${PROJECT_NAME}_gencfg pacecat_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_playback_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_client ${PROJECT_NAME}_playback_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
