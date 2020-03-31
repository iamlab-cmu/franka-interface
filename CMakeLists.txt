cmake_minimum_required (VERSION 3.4)
project (franka-interface
  VERSION 0.0.1 
  LANGUAGES CXX
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(franka-interface)
add_subdirectory(franka-interface-common)
add_subdirectory(libfranka)

add_executable(franka_interface src/franka_interface.cpp)

target_link_libraries(franka_interface 
  franka-interface 
  franka-interface-common
  Franka::Franka
  Eigen3::Eigen3
  ${Boost_LIBRARIES}
)

install(TARGETS franka_interface DESTINATION bin)
