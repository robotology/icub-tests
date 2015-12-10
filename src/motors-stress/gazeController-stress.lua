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
-- e.g., testrunner -v -t gazeController-stress.lua -p "--robot icub" --repetition 3
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
        property:fromArguments(parameter);
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

    if not property:check("robot") then RTF.assertError("The robot name must be given as the test parameter!") end
    robot = property:find("robot"):asString()

    --
    -- open the gazecontrollerclient driver
    --
    local options = yarp.Property()
    options:put("device", "gazecontrollerclient")
    options:put("local", "/posiotion-stress/gaze")
    options:put("remote", "/iKinGazeCtrl")

    gazeDriver = yarp.PolyDriver(options)
    if gazeDriver:isValid() == false then
        RTF.assertError("Cannot open the gazecontrollerclient device")
    end

    -- open the GazeController interface
    iGaze = gazeDriver:viewIGazeControl()
    if iGaze == nil then 
        gazeDriver:close()
        RTF.assertError("Cannot open the IGazeControl interface")
    end

    --
    -- open the remote_controlboard driver
    --
    options:clear()
    options = yarp.Property()
    options:put("device", "remote_controlboard")
    options:put("local", "/posiotion-stress/head")
    options:put("remote", "/"..robot.."/head")

    boardDriver = yarp.PolyDriver(options)
    if boardDriver:isValid() == false then
        RTF.assertError("Cannot open the device")
    end

    imode = boardDriver:viewIControlMode()
    if imode == nil then 
        boardDriver:close()
        RTF.assertError("Cannot open the iControlMode interface")
    end

    --[[
    iamp = boardDriver:viewIAmplifierControl()
    if iamp == nil then 
        driver:close()
        RTF.assertError("Cannot open the IAmplifierControl interface")
    end 
    ]]-- 

    return true
end

--
-- TestCase run()
--
TestCase.run = function()
    local fp = yarp.Vector(3)

    -- x, y and z
    fp:set(0, -1.0)
    fp:set(1, yarp.Random_uniform(-5,5)/10.0)
    fp:set(2, yarp.Random_uniform(0,10)/10.0)
    RTF.testReport("Looking at "..fp:toString(2,-1))
    iGaze:lookAtFixationPoint(fp)    
    --iGaze:waitMotionDone()
    yarp.Time_delay(yarp.Random_uniform(5,30)/10.0)        

    -- checking control mode
    local modes = yarp.IVector(6)
    local ret = imode:getControlModes(modes)
    RTF.testCheck(ret, "Getting the control mode")
    local mode_msg = ""
    local all_mode_pos = true
    for i=0,5 do
        all_mode_pos = all_mode_pos and (yarp.Vocab_decode(modes[i]) ~= "idl")
                                    and (yarp.Vocab_decode(modes[i]) ~= "hwf")
        mode_msg = mode_msg .. yarp.Vocab_decode(modes[i]) .. " "
    end
    RTF.testReport("Control Modes ("..mode_msg..")")
    if not all_mode_pos then
        RTF.assertFail("some of the joints went in idle/hardware fault!")
    end 
end


--
-- TestCase tearDown
--
TestCase.tearDown = function()
    RTF.testReport("Tearing down...")
    -- homing
    if iGaze ~= nil then 
        local fp = yarp.Vector(3)
        fp:set(0, -1.0)
        fp:set(1, 0.0)
        fp:set(2, 0.35) 
        RTF.testCheck(ret, "Homing ...")
        iGaze:lookAtFixationPoint(fp)
        yarp.Time_delay(3.0)
        --iGaze:waitMotionDone()
    end
    gazeDriver:close()
    boardDriver:close()
    yarp.Network_fini()
end

