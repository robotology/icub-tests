--
-- Copyright (C) 2015 iCub Facility
-- Authors: Ali Paikan
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
 
--
-- The TestCase table is used by the lua plugin loader
-- to invoke the corresponding methods:
--
-- TestCase.setup = function(options) ... return true end
-- TestCase.run = function() ... end 
-- TestCase.tearDown = function() ... end 
--
-- The following methods are for reporting, failures or assertions: 
--
-- RTF.setName(name)             : sets the test name (defualt is the test filename)
-- RTF.testReport(msg)           : reports a informative message
-- RTF.testCheck(condition, msg) : reports a failure message
-- RTF.assertError(msg)          : throws an error exception with message
-- RTF.asserFail(msg)            : throws a failure exception with message
-- RTF.getEnvironment()          : returns the test environment params
--

--
-- e.g., testrunner -v -t example-test.lua --param "--example myparam --name MyExampleTest"
--

require("yarp")

function loadParameters(parameter)

    local env = RTF.getEnvironment()
    envprop = yarp.Property()
    envprop:fromArguments(env)
    useSuitContext = envprop:check("context")

    -- load the config file and update the environment if available
    local rf = yarp.ResourceFinder()
    rf:setVerbose(false)
    if useSuitContext then
        rf:setDefaultContext(envprop:find("context"):asString())
    else
        rf:setDefaultContext("RobotTesting")
    end

    -- rf:configure(argc, argv);
    
    local property = yarp.Property()
    local paramprop = yarp.Property()
    paramprop:fromArguments(parameter)

    if paramprop:check("from") then

        local cfgname = paramprop:find("from"):asString()
        if string.len(cfgname) == 0 then
            RTF.assertError("Empty value was set for the '--from' property")
        end

        -- loading configuration file indicated by --from
        local cfgfile = rf:findFileByName(cfgname)
        useTestContext = paramprop:check("context");

        -- if the config file cannot be found from default context or
        -- there is not any context, use the robotname environment as context
        if not useSuitContext and not useTestContext
           and string.len(cfgfile) == 0 and envprop:check("robotname") then
            rf:setContext(envprop:find("robotname"):asString())
            cfgfile = rf:findFileByName(cfgname)
        end

        if string.len(cfgfile) == 0 then
            RTF.assertError("Cannot find configuration file " .. cfgfile)
        end
        -- update the properties with environment
        property:fromConfigFile(cfgfile, envprop);   
    else
        property:fromArguments(parameter)
    end
    return property
end


--
-- Testcase startup()
--
TestCase.setup = function(parameter)   
    -- initialize yarp network
    yarp.Network()

    -- load paramters
    local property = loadParameters(parameter)

    if property:check("name") then
        RTF.setName(property:find("name"):asString())
    end        

    example = property:check("example", yarp.Value("default value")):asString()
    RTF.testReport("Uses "..example.." for the example param!")

    --  do the rest of the initializaiton 
    -- ...

    return true
end

--
-- TestCase run()
--
TestCase.run = function()
    local a = 5
    local b = 3
    RTF.testReport("testing a < b")
    RTF.testCheck(a<b, "a is not smaller than b")

    RTF.testReport("testing a > b")
    RTF.testCheck(a>b, "a is not bigger than b")

    RTF.testReport("testing a == b")
    RTF.testCheck(a==b, "a is not equal to b")
    -- ...
    -- ...
end


--
-- TestCase tearDown
--
TestCase.tearDown = function()
    RTF.testReport("Tearing down...")
    yarp.Network_fini()
end


