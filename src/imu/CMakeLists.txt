cmake_minimum_required(VERSION 3.5)
project(imu)

add_definitions(-D_USE_MATH_DEFINES)
find_package(iDynTree REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RobotTestingFramework_INCLUDE_DIRS})

robottestingframework_add_plugin(${PROJECT_NAME} HEADERS ${PROJECT_NAME}.h
                                                 SOURCES ${PROJECT_NAME}.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} RobotTestingFramework::RTF
                                      RobotTestingFramework::RTF_dll
                                      YARP::YARP_robottestingframework
                                      iDynTree::idyntree-high-level 
                                      iDynTree::idyntree-estimation)

# set the installation options
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)