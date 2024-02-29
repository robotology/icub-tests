add_definitions(-D_USE_MATH_DEFINES)
find_package(iDynTree REQUIRED)
find_package(robometry)

robottestingframework_add_plugin(imu HEADERS imu.h
                                     SOURCES imu.cpp)

# add required libraries
target_link_libraries(imu   RobotTestingFramework::RTF
                            RobotTestingFramework::RTF_dll
                            YARP::YARP_robottestingframework
                            iDynTree::idyntree-high-level 
                            iDynTree::idyntree-estimation
                            robometry::robometry)
# set the installation options
install(TARGETS imu
        EXPORT imu
        COMPONENT runtime
        LIBRARY DESTINATION lib)

install(PROGRAMS move.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)