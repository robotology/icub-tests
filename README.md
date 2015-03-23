icub-tests
==========
The icub-tests repository contains tests for the iCub robot. Tests are written using the Robot Testing Framework (RTF). See https://github.com/robotology/robot-testing for how to use RTF. 


Installing RTF
--------------
* If you have not installed RTF, Please see http://robotology.github.io/robot-testing/index.html. 


Building tests 
--------------
```
    $ git clone https://github.com/robotology/icub-tests.git
    $ cd icub-tests; 
    $ mkdir build; cd build
    $ cmake ../; make
```

Configuration
-------------
* Test cases are built as RTF plug-ins (shared libraries) and can be found in `icub-tests/build/plugins` folder. We need to add the plug-ins path to the platform library environment variable (i.e., `LD_LIBRARY_PATH` on Linux machine). 

```
    $ echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path to icub_tests/build/plugins>' >> ~/.bashrc
```

* Tests configuration (.ini files) can be found in `icub-tests/suit/contexts`. We need to configure `YARP_DATA_DIRS` environment variable so that the test cases can load the configurations.

```
    $ echo 'export YARP_DATA_DIRS=$YARP_DATA_DIRS:<path to icub-tests/suit>' >> ~/.bashrc
```

Running the tests
-----------------
You can run the tests by using the RTF `testrunner` and the test suits XML files. For example to run the camera tests for the iCub simulator:

```
    $ cd icub-tests/suits
    $ testrunner --verbose --suit cameras-icubSim.xml -o camera-result.txt
```

Writing new test cases
----------------------
* create a folder with the name of your test case in the `icub-tests/src/` folder to keep your test codes: 

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
    string example = property.check("example", Value("default value")).asString();

    RTF_REPORT(Asserter::format("Use '%s' for the example param!",
                                       example.c_str()));
    return true;
}

void ExampleTest::tearDown() {
    // finalization goes here ...
}

void ExampleTest::run() {

    int a = 5; int b = 3;
    RTF_REPORT("testing a < b");
    RTF_CHECK(a<b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_REPORT("testing a > b");
    RTF_CHECK(a>b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_REPORT("testing a == b");
    RTF_CHECK(a==b, Asserter::format("%d is not smaller than %d.", a, b));
    // add more 
    // ...
}
```

Notice: The `RTF_CHECK`, `RTF_REPORT` do NOT threw any exception and are used to add failure or report messages to the result collector. Instead, all the macros which include `_ASSERT_` within their names (e.g., `RTF_ASSERT_FAIL`) throw exceptions which stop only the current test case (Not the whole test suite) of being proceed. The error/failure message thrown by the exception are caught. (See http://robotology.github.io/robot-testing/documentation/TestAssert_8h.html for basic assertion macros). 

All the report/assertion macros include the source line number where the check/report or assertion happen. To see them, you can run the test case or suit with `--detail` parameter using the `testrunner` (See http://robotology.github.io/robot-testing/documentation/testrunner.html). 

* Create a cmake file to build the plugin: 

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


 




