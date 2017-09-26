cmake_minimum_required(VERSION 2.8.12)
project(polhemus_ros_driver)

add_compile_options(-std=c++11)

if(USB_LIBRARIES AND USB_INCLUDE_DIRS)
  # in cache already
  set(USB_FOUND TRUE)
else(USB_LIBRARIES AND USB_INCLUDE_DIRS)
  find_path(USB_INCLUDE_DIR
    NAMES
    usb.h
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    /sw/include
  )

  find_library(USB_LIBRARY
    NAMES
    usb
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
  )

  set(USB_INCLUDE_DIRS ${USB_INCLUDE_DIR})
  set(USB_LIBRARIES ${USB_LIBRARY})

  if(USB_INCLUDE_DIRS AND USB_LIBRARIES)
    set(USB_FOUND TRUE)
  endif(USB_INCLUDE_DIRS AND USB_LIBRARIES)

  if(USB_FOUND)
    if(NOT USB_FIND_QUIETLY)
      message(STATUS "Found USB: ${USB_LIBRARIES}")
    endif(NOT USB_FIND_QUIETLY)
  else(USB_FOUND)
    if(USB_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find USB")
    endif(USB_FIND_REQUIRED)
  endif(USB_FOUND)

  # show the USB_INCLUDE_DIRS and USB_LIBRARIES variables only in the advanced view
  mark_as_advanced(USB_INCLUDE_DIRS USB_LIBRARIES)
endif(USB_LIBRARIES AND USB_INCLUDE_DIRS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2
  tf2_ros
  roscpp
)

catkin_package(
  LIBRARIES polhemus_liberty
  CATKIN_DEPENDS geometry_msgs tf2 tf2_ros roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USB_INCLUDE_DIRS}
)

add_library(polhemus_liberty
  src/liberty.cpp
)

add_executable(polhemus_tf_broadcaster src/polhemus_tf_broadcaster.cpp)
target_link_libraries(polhemus_tf_broadcaster
  ${catkin_LIBRARIES}
  ${USB_LIBRARIES}
  polhemus_liberty
)

install(TARGETS polhemus_tf_broadcaster polhemus_liberty
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/start.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
