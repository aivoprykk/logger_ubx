cmake_minimum_required(VERSION 3.16)

# Set extra component dirs to include logger_ubx and dependencies
set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_common"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_config"
    "${CMAKE_CURRENT_LIST_DIR}/../../../sconfig"
    "${CMAKE_CURRENT_LIST_DIR}/../../../strutil"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ubx_message_handler)

# Suppress unused-const-variable warnings for logger_ubx component (example build)
idf_component_get_property(logger_ubx_lib logger_ubx COMPONENT_LIB)
target_compile_options(${logger_ubx_lib} PRIVATE -Wno-unused-const-variable -Wno-unused-function)
