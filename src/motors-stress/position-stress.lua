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
-- e.g., testrunner -v -t position-stress.lua -e "--robotname icub" -p "--from motor_stress_head.ini" --repetition 3
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
        property:fromString(parameter);
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
    if not property:check("part") then RTF.assertError("The part name must be given as the test parameter!") end
    if not property:check("joints") then RTF.assertError("The joints list must be given as the test parameter!") end
    if not property:check("min") then RTF.assertError("The joints min position list must be given as the test parameter!") end
    if not property:check("max") then RTF.assertError("The joints max postion list must be given as the test parameter!") end
    if not property:check("speed") then RTF.assertError("The joints speed list must be given as the test parameter!") end

    part = property:find("part"):asString()
    robot = property:find("robot"):asString()
    joints = property:find("joints"):asList()
    minpos = property:find("min"):asList()
    maxpos = property:find("max"):asList()
    speed = property:find("speed"):asList()
    if joints == nil then RTF.assertError("joints parameter are not given as a list") end
    if minpos == nil then RTF.assertError("max parameter are not given as a list") end
    if maxpos == nil then RTF.assertError("min parameter are not given as a list") end
    if speed == nil then RTF.assertError("speed parameter are not given as a list") end
    
    options = yarp.Property()
    options:put("device", "remote_controlboard")
    options:put("local", "/posiotion-stress/"..part)
    options:put("remote", "/"..robot.."/"..part)

    -- setting QoS pereferences for the driver (local and remote)
    qos_local = options:addGroup("local_qos")
    qos_local:put("thread_priority", 20)
    qos_local:put("thread_policy", 1)
    qos_local:put("packet_priority", "LEVEL:HIGH")

    qos_remote = options:addGroup("remote_qos")
    qos_remote:put("thread_priority", 20)
    qos_remote:put("thread_policy", 1)
    qos_remote:put("packet_priority", "LEVEL:HIGH")

    -- open the driver
    driver = yarp.PolyDriver(options)
    if driver:isValid() == false then
        RTF.assertError("Cannot open the device")
    end

    -- open the interfaces
    ipos = driver:viewIPositionControl()
    if ipos == nil then 
        driver:close()
        RTF.assertError("Cannot open the IPositionControl interface")
    end

    ilimit = driver:viewIControlLimits()
    if ilimit == nil then 
        driver:close()
        RTF.assertError("Cannot open the IControlLimits interface")
    end

    ienc = driver:viewIEncoders()
    if ienc == nil then 
        driver:close()
        RTF.assertError("Cannot open the IEncoders interface")
    end

    imode = driver:viewIControlMode()
    if imode == nil then 
        driver:close()
        RTF.assertError("Cannot open the IControlMode interface")
    end
    
    iamp = driver:viewIAmplifierControl()
    if iamp == nil then 
        driver:close()
        RTF.assertError("Cannot open the IAmplifierControl interface")
    end

    return true
end

--
-- TestCase run()
--
TestCase.run = function()
    local axes = ipos:getAxes()
    local modes = yarp.IVector(axes)
    local amps = yarp.Vector(axes)
    for i=0,joints:size()-1 do
        local home = ienc:getEncoder(i)
        -- set the control mode
        ipos:setPositionMode()
        ipos:setRefSpeed(i, speed:get(i):asInt())

        -- move the joint 
        RTF.testReport("setting position of joint ".. i.." to min : "..minpos:get(i):asDouble())
        ipos:positionMove(i, minpos:get(i):asDouble())  
        yarp.Time_delay(yarp.Random_uniform(2,20)/10.0)        
        iamp:getCurrents(amps:data())
        RTF.testReport("Currents: "..amps:toString(-1, -1))

        RTF.testReport("setting position of joint ".. i.." to max : "..maxpos:get(i):asDouble())
        ipos:positionMove(i, maxpos:get(i):asDouble())
        yarp.Time_delay(yarp.Random_uniform(2,20)/10.0)
        iamp:getCurrents(amps:data())
        RTF.testReport("Currents: "..amps:toString(-1, -1))

        RTF.testReport("setting position of joint ".. i.." to home : "..home)
        ipos:positionMove(i, home)
        yarp.Time_delay(yarp.Random_uniform(2,20)/10.0)
        iamp:getCurrents(amps:data())
        RTF.testReport("Currents: "..amps:toString(-1, -1))

        -- checking control mode
        local ret = imode:getControlModes(modes)
        RTF.testCheck(ret, "Cannot get the control mode")
        local mode_msg = ""
        local all_mode_pos = true
        for i=0,axes-1 do
            all_mode_pos = all_mode_pos and (yarp.Vocab_decode(modes[i]) == "pos")
            mode_msg = mode_msg .. yarp.Vocab_decode(modes[i]) .. " "
        end
        RTF.testReport("Control Modes ("..mode_msg..")")
        if not all_mode_pos then
            RTF.assertFail("some of the joints went in idle/hardware fault!")
        end 
    end 
    RTF.testReport("")
end


--
-- TestCase tearDown
--
TestCase.tearDown = function()
    RTF.testReport("Tearing down...")
    driver:close()
    yarp.Network_fini()
end


