cmake_minimum_required(VERSION 3.10)
project(footbot_specialization)

# Set the build type if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

# Check for Lua and add include path
find_package(Lua53 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

# Manually specify ARGoS paths (adjust these paths if necessary)
set(ARGOS_INCLUDE_DIRS "/usr/local/include/argos3")  # Ajusta la ruta según donde esté instalado ARGoS
set(ARGOS_LIBRARY_DIRS "/usr/local/lib/argos3")  # Ajusta la ruta según donde esté instalado ARGoS

# Add ARGoS paths
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})


# Define the library for your controller AND loop functions.
# We list all the .cpp files that need to be compiled into a single shared library.
# ARGoS will then load this library and find both the controller and the
# loop functions by their registered names.

add_library(footbot_specialization SHARED
  footbot_specialization.cpp      
  footbot_specialization.h    
  specialization_loop_functions.cpp  
  specialization_loop_functions.h
)



# Link against the required libraries
target_link_libraries(footbot_specialization
  ${ARGOS_LIBRARY_DIRS}/libargos3core_simulator.so  # ARGoS core simulator library
  ${ARGOS_LIBRARY_DIRS}/libargos3plugin_simulator_footbot.so  # Foot-bot simulator plugin
  ${ARGOS_LIBRARY_DIRS}/libargos3plugin_simulator_media.so  # For range-and-bearing and LEDs to work
  ${ARGOS_LIBRARY_DIRS}/libargos3plugin_simulator_qtopengl.so  # Only needed if using QTUserFunctions
  ${LUA_LIBRARIES}  # Lua libraries
)

# Set proper compilation flags for better debugging
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(footbot_specialization PRIVATE -g -O0)
endif()