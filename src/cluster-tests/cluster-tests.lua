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
-- setup is called before the test's run to setup 
-- the user defined fixture
-- @return Boolean (true/false uppon success or failure)
--
hosts = {}

function runRemoteCommand(host, cmd)
    local file = io.popen('ssh '..host..' '..cmd, 'r')
    local output = file:read('*all')
    file:close()
    return output
end

function checkPriorityLimits(host) 
    RTF.testReport("Testing process priority limits (99)")
    local result = runRemoteCommand(host, "ulimit -r")
    if result:len() == 0 then
        RTF.testCheck(result:len() > 0, "cannot run 'ulimit -r' on '"..host.."'")
    else
        RTF.testCheck(tonumber(result) == 99, "process prioirty is limited ("..tonumber(result)..")")
    end 
end

function checkUDPLimits(host) 
    RTF.testReport("Testing UDP maximum read buffer limits (8388608)")
    local result = runRemoteCommand(host, "cat /proc/sys/net/core/rmem_max")
    if result:len() == 0 then
        RTF.testCheck(result:len() > 0, "cannot run 'cat /proc/sys/net/core/rmem_max' on '"..host.."'")
    else
        RTF.testCheck(tonumber(result) == 8388608, "UDP maximum read buffer size is limited ("..tonumber(result)..")")
    end 
end

--
-- Testcase srtup()
--
TestCase.setup = function(parameter)   
    RTF.setName("Cluster tests")
    -- get the hosts list in cluster
    for host in string.gmatch(parameter, "[^%s]+") do
       hosts[#hosts+1] = host
    end
    if #hosts == 0 then
        RTF.assertError("no host is given! Please provide the hosts name as paramter (e.g. -p 'pc104 icub16').")
    end
    return true
end

--
-- TestCase run()
--
TestCase.run = function()
    for i=1,#hosts do
        local host = hosts[i]
        RTF.testReport("")
        RTF.testReport("Checking host ["..host.."] ...")
        checkPriorityLimits(host)
        checkUDPLimits(host)
    end
end


--
-- TestCase tearDown
--
TestCase.tearDown = function()
    RTF.testReport("Tearing down...")
end

