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
Please check the `icub-tests/example` folder for a template for developing tests for the iCub. 


 




