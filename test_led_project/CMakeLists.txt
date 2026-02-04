cmake_minimum_required(VERSION 3.5)

# Incluir el componente de micro-ROS
set(EXTRA_COMPONENT_DIRS components/micro_ros_espidf_component)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(test_led_microros)
