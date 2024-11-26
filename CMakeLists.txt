################################################################################
cmake_minimum_required(VERSION 3.8)
project(UnifiedParticle)
set(CMAKE_CXX_STANDARD 11)#we use C++ 11
#this setting will put all the generated exe, lib, bin into some certain folders
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
find_package(OpenMP REQUIRED)
if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
   # string( REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}") #enable assertion in release with debug mode;
endif()
include_directories(HLBFGS)
include_directories(ANN)
add_subdirectory(src)
















