# ==============================================================================
# Biofloc Firmware ROS - Root CMakeLists.txt
# ESP-IDF v5.3 + micro-ROS Jazzy
# ==============================================================================
cmake_minimum_required(VERSION 3.16)

# Incluir el componente micro-ROS antes de project()
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components/micro_ros_espidf_component
)

# Incluir el sistema de build de ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Nombre del proyecto
project(biofloc_firmware_ros)
