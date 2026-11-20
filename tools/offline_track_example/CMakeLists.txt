cmake_minimum_required(VERSION 3.5)
project(aubo_offline_track_example CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point this at the aubo_driver package (contains include/ and lib/lib64/).
set(AUBO_DRIVER_DIR "$ENV{HOME}/aubo_ws/src/aubo_i10_ros_noetic/aubo_driver"
    CACHE PATH "Path to the aubo_driver package")

set(AUBO_LIB_DIR  "${AUBO_DRIVER_DIR}/lib/lib64/aubocontroller")
set(CFG_LIB_DIR   "${AUBO_DRIVER_DIR}/lib/lib64/config")
set(LOG_LIB_DIR   "${AUBO_DRIVER_DIR}/lib/lib64/log4cplus")

if(NOT EXISTS "${AUBO_DRIVER_DIR}/include/aubo_driver/serviceinterface.h")
  message(FATAL_ERROR "serviceinterface.h not found under ${AUBO_DRIVER_DIR}.
  Configure with: cmake -DAUBO_DRIVER_DIR=/path/to/aubo_driver ..")
endif()

find_library(AUBO_LIB auborobotcontroller PATHS "${AUBO_LIB_DIR}" NO_DEFAULT_PATH)
if(NOT AUBO_LIB)
  message(FATAL_ERROR "libauborobotcontroller not found in ${AUBO_LIB_DIR}")
endif()

find_library(CONFIG_LIB config    PATHS "${CFG_LIB_DIR}" NO_DEFAULT_PATH)
find_library(LOG4CPLUS_LIB log4cplus PATHS "${LOG_LIB_DIR}" NO_DEFAULT_PATH)

add_executable(offline_track src/main.cpp)
target_include_directories(offline_track PRIVATE "${AUBO_DRIVER_DIR}/include")
target_link_libraries(offline_track PRIVATE ${AUBO_LIB} ${CONFIG_LIB} ${LOG4CPLUS_LIB} pthread)

# Bake the vendor library paths into the binary so no LD_LIBRARY_PATH is needed.
set_target_properties(offline_track PROPERTIES
  BUILD_RPATH   "${AUBO_LIB_DIR};${CFG_LIB_DIR};${LOG_LIB_DIR}"
  INSTALL_RPATH "${AUBO_LIB_DIR};${CFG_LIB_DIR};${LOG_LIB_DIR}")
