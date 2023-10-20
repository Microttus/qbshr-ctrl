cmake_minimum_required(VERSION 3.8)
project(qbshr_ctr
        VERSION 0.0.1
        DESCRIPTION "Library to interface with qbRobotics Soft Hand Research"
        LANGUAGES CXX C
)

message(${CMAKE_PROJECT_NAME})

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

##Include directories
include_directories(include/${CMAKE_PROJECT_NAME})

##Library
add_library(${CMAKE_PROJECT_NAME} STATIC
        #src/robotHandControl.cc
        src/qbSoftHandHandler.cc
        src/qbSoftHandControl.cc
        include/qbshr_ctr/qbSoftHandControl.hh
        include/qbshr_ctr/qbSoftHandHandler.hh
)

add_executable(qbshr_test src/simpTest.cc)

ament_export_targets(${CMAKE_PROJECT_NAME} HAS_LIBRARY_TARGET)

target_include_directories(qbshr_ctr PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

##Directory
add_subdirectory(libs/qbdevice-api-7.x.x qbrobotics-device)


target_link_libraries(${CMAKE_PROJECT_NAME}
        PRIVATE
        qbrobotics_driver::qbrobotics_driver
        #Serial::Serial
)


#target_link_libraries(qbshr_test
#        PRIVATE
#        qbrobotics_driver::qbrobotics_driver
#)

ament_target_dependencies(qbshr_test
        rclcpp
)

install(TARGETS
        qbshr_test
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/${CMAKE_PROJECT_NAME}
        DESTINATION include
        #FILES_MATCHING PATTERN "*.hh"
)

# Export package in order to make an indeclinable library
install(TARGETS ${CMAKE_PROJECT_NAME} qbrobotics_driver Serial
        EXPORT ${CMAKE_PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


ament_export_include_directories(include/${CMAKE_PROJECT_NAME})# libs)
ament_export_libraries(${CMAKE_PROJECT_NAME} qbrobotics_driver Serial)
ament_export_targets(${CMAKE_PROJECT_NAME})

ament_export_dependencies(ament_cmake)

ament_package()