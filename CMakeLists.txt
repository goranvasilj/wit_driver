cmake_minimum_required(VERSION 2.8.3)
project(wit_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
 tf
  message_generation
)

add_service_files(FILES modbus_srv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    serial
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wit_driver src/wit_driver.cpp)
add_executable(modbus_master_serial src/modbus_master_serial.cpp)
add_executable(wit_driver_fast src/wit_driver_fast.cpp)
add_executable(modbus_master_serial_fast src/modbus_master_serial_fast.cpp)


target_link_libraries(wit_driver
  ${catkin_LIBRARIES}
)
target_link_libraries(modbus_master_serial
  ${catkin_LIBRARIES}
)
target_link_libraries(wit_driver_fast
  ${catkin_LIBRARIES}
)
target_link_libraries(modbus_master_serial_fast
  ${catkin_LIBRARIES}
)
