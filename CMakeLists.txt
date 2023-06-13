cmake_minimum_required(VERSION 2.8.3)
project(adi_driver)

#add_compile_options(-std=c++11)

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# find and include boost
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# this project does not use catkin anymore
include_directories(include)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(adis16470_node src/adis16470_node.cpp src/adis1647x.cpp src/serial_port.cpp)
#target_link_libraries(adis16470_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# no catkin
target_link_libraries(adis16470_node PRIVATE Threads::Threads)
target_link_libraries(adis16470_node LINK_PUBLIC ${Boost_LIBRARIES})

#Add adis16495_node
#install(TARGETS adis16470_node adxl345_node adis16495_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
# install in a sensible place
install(TARGETS adis16470_node
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)