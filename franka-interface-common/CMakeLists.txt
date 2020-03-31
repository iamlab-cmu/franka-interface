cmake_minimum_required (VERSION 3.4)
project (franka-interface-common
  VERSION 0.0.1 
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost COMPONENTS REQUIRED filesystem system thread)

include_directories(${Boost_INCLUDE_DIRS})

## Library
add_library(franka-interface-common SHARED
  src/franka_interface_state_info.cpp
  src/run_loop_process_info.cpp
  src/SharedMemoryInfo.cpp
)

# Add namespace
# add_library(FrankaInterface::FrankaInterface ALIAS franka-interface)

target_link_libraries(franka-interface-common PUBLIC
  Threads::Threads
  ${Boost_LIBRARIES}
  rt
)


target_include_directories(franka-interface-common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(franka-interface-common PRIVATE src)


## Installation
include(GNUInstallDirs)
install(TARGETS franka-interface-common
  EXPORT franka-interface-common
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/franka-interface-common"
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
