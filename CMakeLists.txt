cmake_minimum_required(VERSION 3.6.0 FATAL_ERROR)
project(SocialNetwork)

include_directories(include)
 
file(GLOB SOURCES "src/*.cpp")
 
add_executable(SocialNetwork ${SOURCES})
set_property(TARGET SocialNetwork PROPERTY CXX_STANDARD 11)
set_property(TARGET SocialNetwork PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT SocialNetwork)