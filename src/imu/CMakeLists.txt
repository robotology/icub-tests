add_definitions(-D_USE_MATH_DEFINES)

robottestingframework_add_plugin(imu HEADERS imu.h
                                     SOURCES imu.cpp)

# add required libraries
target_link_libraries(imu RobotTestingFramework::RTF
                                      RobotTestingFramework::RTF_dll
                                      YARP::YARP_robottestingframework
                                      iDynTree::idyntree-high-level 
                                      iDynTree::idyntree-estimation)
# set the installation options
install(TARGETS imu
        EXPORT imu
        COMPONENT runtime
        LIBRARY DESTINATION lib)