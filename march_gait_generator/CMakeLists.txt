cmake_minimum_required(VERSION 2.8.3)
project(march_gait_generator)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

## Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
        src/GaitGenerator.cpp
        src/main.cpp
        src/widgets/FancySlider.cpp
        )

## Add the "march_gait_generator" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(march_gait_generator ${SRC_FILES})

## Link the march_gait_generator executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(march_gait_generator ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install
install(TARGETS march_gait_generator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
