file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

set(G2O_LIBS
  /opt/ros/noetic/lib/libg2o_types_slam3d.so
  /opt/ros/noetic/lib/libg2o_core.so
  /opt/ros/noetic/lib/libg2o_solver_cholmod.so
  /opt/ros/noetic/lib/libg2o_solver_csparse.so
  /opt/ros/noetic/lib/libg2o_solver_pcg.so
  )

add_executable(
  ${PROJECT_NAME}
  ${SOURCES}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_LIBS}
)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
