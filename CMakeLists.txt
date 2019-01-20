cmake_minimum_required(VERSION 2.8)
project(kf)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)
find_package(OpenCV)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

# add the subscriber 
add_executable(img_sub src/img_sub.cpp)
add_dependencies(img_sub ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(img_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ekf src/ekf.cpp)
add_dependencies(ekf ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ekf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )


install(TARGETS img_sub ekf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


