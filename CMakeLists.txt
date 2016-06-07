cmake_minimum_required (VERSION 2.6)
project (camera_fake)

add_executable (base_aria src/main.cpp)
target_link_libraries (base_aria tiobj tisys Aria)

install(TARGETS base_aria DESTINATION bin)
