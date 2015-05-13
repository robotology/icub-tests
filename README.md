icub-tests
==========
The `icub-tests` repository contains tests for the iCub robot. Tests are written using the Robot Testing Framework (RTF). 
See https://github.com/robotology/robot-testing for how to use RTF. 


### Table of Contents
**[Installing RTF](#installing-rtf)** 

**[Building tests](#building-tests)** 

**[Configuration](#configuration)** 

**[Writing new test cases](#writing-new-test-cases)** 

**[Running a single test case](#running-a-single-test-case)** 

**[Running multiple tests using test suite](#running-multiple-tests-using-test-suite)** 


## Installing RTF

If you have not installed RTF, Please see http://robotology.github.io/robot-testing/index.html . 
Note that `icub-tests` use the YARP support in RTF, so please check that RTF is compiled with the `ENABLE_MIDDLEWARE_PLUGINS` CMake option setted to ON.


## Building tests 

On Linux machines, open a terminal and type:
```
    $ git clone https://github.com/robotology/icub-tests.git
    $ cd icub-tests; 
    $ mkdir build; cd build
    $ cmake ../; make
```
On windows machines use the CMake program to create Visual Studio project and build it. 

Note: in case CMake fails to detect the RTF package check that you installed RTF or that you set the RTF_DIR environment variables to point to the build directory. Also check that RTF was compiled with YARP support enables (set `ENABLE_MIDDLEWARE_PLUGINS` and verify that `ENABLE_YARP` is ON).

## Configuration

* Test cases are built as RTF plug-ins (shared libraries) and can be found in `icub-tests/build/plugins` folder. We need to add the plug-ins path to the platform library environment variable (i.e., `LD_LIBRARY_PATH` on Linux machine). 

```
    $ echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path to icub_tests/build/plugins>' >> ~/.bashrc
```

* Tests configuration (.ini files) can be found in `icub-tests/suit/contexts`. We need to configure `YARP_DATA_DIRS` environment variable so that the test cases can load the configuration files.

```
    $ echo 'export YARP_DATA_DIRS=$YARP_DATA_DIRS:<path to icub-tests/suit>' >> ~/.bashrc
```


## Writing new test cases

* Create a folder with the name of your test case in the `icub-tests/src/` folder to keep your test codes: 

```
    $ mkdir icub-tests/src/example-test
```

* Create a child test class inherited from the `YarpTestCase`:

```c++
#ifndef _CAMERATEST_H_
#define _CAMERATEST_H_

#include <YarpTestCase.h>

class ExampleTest : public YarpTestCase {
public:
    ExampleTest();
    virtual ~ExampleTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
};

#endif //_CAMERATEST_H
```

* Implement the test case: 

```c++
#include <Plugin.h>
#include "ExampleTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;

// prepare the plugin
PREPARE_PLUGIN(ExampleTest)

ExampleTest::ExampleTest() : YarpTestCase("ExampleTest") {
}

ExampleTest::~ExampleTest() { }

bool ExampleTest::setup(yarp::os::Property &property) {

    // initialization goes here ...
    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());
   
    string example = property.check("example", Value("default value")).asString();

    RTF_TEST_REPORT(Asserter::format("Use '%s' for the example param!",
                                       example.c_str()));
    return true;
}

void ExampleTest::tearDown() {
    // finalization goes here ...
}

void ExampleTest::run() {

    int a = 5; int b = 3;
    RTF_TEST_REPORT("testing a < b");
    RTF_TEST_CHECK(a<b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_TEST_REPORT("testing a > b");
    RTF_TEST_CHECK(a>b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_TEST_REPORT("testing a == b");
    RTF_TEST_CHECK(a==b, Asserter::format("%d is not smaller than %d.", a, b));
    // add more 
    // ...
}
```

Notice: The `RTF_TEST_CHECK`, `RTF_TEST_REPORT` do NOT threw any exception and are used to add failure or report messages to the result collector. Instead, all the macros which include `_ASSERT_` within their names (e.g., `RTF_ASSERT_FAIL`) throw exceptions which prevent only the current test case (Not the whole test suite) of being proceed. The error/failure messages thrown by the exceptions are caught. (See [*Basic Assertion macros*](http://robotology.github.io/robot-testing/documentation/TestAssert_8h.html)). 

The report/assertion macros store the source line number where the check/report or assertion happen. To see them, you can run the test case or suit with `--detail` parameter using the `testrunner` (See [*Running test case plug-ins using testrunner*](http://robotology.github.io/robot-testing/documentation/testrunner.html)). 

* Create a cmake file to build the plug-in: 

```cmake
cmake_minimum_required(VERSION 2.8.9)

# set the project name
set(PROJECTNAME ExampleTest)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF)
find_package(RTF COMPONENTS DLL)
find_package(YARP)

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# add required libraries 
link_libraries(${RTF_LIBRARIES}
               ${YARP_LIBRARIES})

# add the source codes to build the plugin library
add_library(${PROJECTNAME} MODULE ExampleTest.h
                                  ExampleTest.cpp)

# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
```

* Call your cmake file from the `icub-test/CMakeLists.txt` to build it along with the other other test plugins.
To do that, adds the following line to the `icub-test/CMakeLists.txt`

```
    # Build example test 
    add_subdirectory(src/example-test)
```

Please check the `icub-tests/example` folder for a template for developing tests for the iCub. 

## Running a single test case

As it is documented here ([*Running test case plug-ins using testrunner*](http://robotology.github.io/robot-testing/documentation/testrunner.html)) 
you can run a single test case or run it with the other tests using a test suite.  For example, to run a single test case: 

```
    testrunner --verbose --test plugins/ExampleTest.so  --param "--name MyExampleTest"
```

or to run the iCubSim camera test whith the test configuration file:

```
    testrunner --verbose --test plugins/CameraTest.so --param "--from right_camera.ini" --environment "--robotname icubSim"
```

This runs the icubSim right-camera test with the parameters specified in the `right_camera.ini` which can be found in `icub-tests/suits/contexts/icubSim` folder. 
Notice that the environment parameter `--robotname icubSim` is used to locate the correct context (for this examples is `icubSim`) and also to update the variables loaded from the `right_camera.ini` file. 


## Running multiple tests using a test suite

You can update one of the existing suite XML files to add your test case plug-in and its parameters or create a new test suite which keeps all the relevant test cases. 
For example the `basic-icubSim.xml` test suite keeps the basic tests for cameras and motors: 

```xml
<?xml version="1.0" encoding="UTF-8"?>

<suit name="Basic Tests Suite">
    <description>Testing robot's basic features</description>
    <environment>--robotname icubSim</environment>
    <fixture param="--fixture icubsim-fixture.xml"> yarpmanager </fixture>

    <!-- Camera -->
    <test type="dll" param="--from right_camera.ini"> CameraTest </test>
    <test type="dll" param="--from left_camera.ini"> CameraTest </test> 
    
    <!-- Motors -->
    <test type="dll" param="--from test_right_arm.ini"> MotorTest </test>
    <test type="dll" param="--from test_left_arm.ini"> MotorTest </test>
</suit>

```

Then you can run all the test cases from the test suite: 

```
    testrunner --verbose --suit icub-tests/suits/basics-icubSim.xml
```

The `testrunner`, first, launches the iCub simulator and then runs all the tests one after each other. After running all the test cases, the `tesrunner` stop the simulator. If the 
iCub simulator crashes during the test run, the `testrunner` re-launchs it and continues running the remaining tests. 

How `testrunner` knows that it should launch the iCub simulator before running the tests? Well, this is indicated by `<fixture param="--fixture icubsim-fixture.xml"> yarpmanager </fixture>`. 
The `testrunner` uses the `yarpmanager` fixture plug-in to launch the modules which are listed in the `icubsim-fixture.xml`.  Notice that all the fixture files should be located in the `icub-tests/suits/fixtures` folder. 

## Contributers

* Ali Paikan 
* Lorenzo Natale
* Silvio Traversaro
* Alessandro Scalzo
* Marco Randazzo

