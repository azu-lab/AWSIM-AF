cmake_minimum_required(VERSION 3.5)

project(awsim_client VERSION 0.1 LANGUAGES CXX)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(Qt6 REQUIRED COMPONENTS Network)
find_package(Qt6 REQUIRED Core Widgets WebEngineWidgets)
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)
find_package(GLUT REQUIRED)

#find_package(PCL 1.12 REQUIRED COMPONENTS common io)
#find_package(rclcpp REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(rcl_interfaces REQUIRED)


set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        mgrs_projector.hpp
        mgrs_projector.cpp
        scenario_datatype.h
        point_cloud_widget.h
        point_cloud_widget.cpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(awsim_client
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        point_cloud_merge.cpp
        point_cloud_merge.h
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET awsim_client APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(awsim_client SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(awsim_client
            ${PROJECT_SOURCES}
        )
    endif()
endif()

include_directories(/opt/ros/humble/include/pcl_msgs)
include_directories(/usr/include/pcl-1.12)
include_directories(/opt/ros/humble/include/message_filters)
include_directories(/opt/ros/humble/include/rcl)
include_directories(/usr/local/include/freetype2)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/statistics_msgs)
include_directories(/opt/ros/humble/include/libstatistics_collector)
include_directories(/opt/ros/humble/include/tracetools)
include_directories(/opt/ros/humble/include/rcutils)
include_directories(/opt/ros/humble/include/rosidl_runtime_c)
include_directories(/opt/ros/humble/include/rcl_yaml_param_parser)
include_directories(/opt/ros/humble/include/rosidl_runtime_cpp)
include_directories(/opt/ros/humble/include/builtin_interfaces)
include_directories(/opt/ros/humble/include/rcpputils)
include_directories(/opt/ros/humble/include/rosidl_typesupport_interface)
include_directories(/opt/ros/humble/include/rmw)
include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/sensor_msgs)
include_directories(/opt/ros/humble/include/rcl_interfaces)
include_directories(/opt/ros/humble/include)
include_directories(yaml_cpp/include)
include_directories(/home/zzhhaa1/Desktop/awsim_client/lanelet2/include)
include_directories(/usr/include/eigen3)
include_directories(/home/zzhhaa1/Desktop/awsim_client/Geographic/include)
include_directories(/opt/ros/humble/include/tf2_ros/)
include_directories(/opt/ros/humble/include/tf2/)
include_directories(/opt/ros/humble/include/geometry_msgs/)
include_directories(/opt/ros/humble/include/tf2_msgs/)
include_directories(/opt/ros/humble/include/diagnostic_msgs/)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(yaml_cpp/lib)
link_directories(/opt/ros/humble/lib)
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
)


target_link_libraries(awsim_client PRIVATE libyaml-cpp.a)
target_link_libraries(awsim_client PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(awsim_client PRIVATE Qt6::Network)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/lanelet2/lib/liblanelet2_core.so.1.2.1)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/lanelet2/lib/liblanelet2_io.so.1.2.1)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/lanelet2/lib/liblanelet2_projection.so.1.2.1)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/lanelet2/lib/liblanelet2_matching.so.1.2.1)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/lanelet2/lib/liblanelet2_routing.so.1.2.1)
target_link_libraries(awsim_client PRIVATE /home/zzhhaa1/Desktop/awsim_client/Geographic/lib/libGeographicLib.so.26.0.0)
target_link_libraries(awsim_client PRIVATE Qt6::Core Qt6::Widgets Qt6::WebEngineWidgets)
target_link_libraries(awsim_client PRIVATE Qt6::OpenGLWidgets)
target_link_libraries(awsim_client PRIVATE ${rclcpp_LIBRARIES})
target_link_libraries(awsim_client PRIVATE ${sensor_msgs_LIBRARIES})
target_link_libraries(awsim_client PRIVATE ${rcl_interfaces_LIBRARIES})
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/librcl.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/librclcpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libtracetools.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/librmw.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/librcutils.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/liblibstatistics_collector.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/librosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /usr/lib/x86_64-linux-gnu/libglut.so.3)
#target_link_libraries(awsim_client PRIVATE /usr/local/lib/libfreetype.so.6.20.1)
target_link_libraries(awsim_client PRIVATE /usr/lib/x86_64-linux-gnu/libpcl_io.so)
target_link_libraries(awsim_client PRIVATE /usr/lib/x86_64-linux-gnu/libpcl_ml.so)
target_link_libraries(awsim_client PRIVATE /usr/lib/x86_64-linux-gnu/libpcl_common.so)
target_link_libraries(awsim_client PRIVATE ${PCL_LIBRARIES})
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libpcl_ros_tf.a)
target_link_libraries(awsim_client PRIVATE /usr/lib/x86_64-linux-gnu/libboost_filesystem.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libtf2_ros.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libdiagnostic_msgs__rosidl_typesupport_cpp.so)
target_link_libraries(awsim_client PRIVATE /opt/ros/humble/lib/libmessage_filters.so)

set_target_properties(awsim_client PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS awsim_client
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(awsim_client)
endif()
