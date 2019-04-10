cmake_minimum_required(VERSION 2.8.3)
project(march_gait_generator)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        rviz
        roscpp
        LIBRARIES
        march_gait_generator
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "include/*.h"
)

roslint_cpp(${lintfiles})

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


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(include ${catkin_LIBRARY_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

add_library(march_gait_generator
        include/${PROJECT_NAME}/Gait.h
        include/${PROJECT_NAME}/GaitGenerator.h
        include/${PROJECT_NAME}/Pose.h
        include/${PROJECT_NAME}/PoseStamp.h
        include/${PROJECT_NAME}/widgets/FancySlider.h
        src/Gait.cpp
        src/GaitGenerator.cpp
        src/Pose.cpp
        src/widgets/FancySlider.cpp
        src/main.cpp
        )

target_link_libraries(march_gait_generator ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(gait_generator_dummy src/main.cpp)
target_link_libraries(gait_generator_dummy march_gait_generator)


install(TARGETS march_gait_generator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test
            test/march_gait_generator.test
            test/TestRunner.cpp
            test/TestPose.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_gait_generator gtest gmock)
endif ()
