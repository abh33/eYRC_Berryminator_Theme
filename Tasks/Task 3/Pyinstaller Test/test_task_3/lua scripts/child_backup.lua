function sysCall_init()
    -- do some initialization here
    
    s = (sim.getStringSignal("points"))
    print(s)
    s = mysplit(s, "%%")
    print(s)
    
    target_point_1 = {tonumber(s[1]), tonumber(s[2])}
    print("target_point_1 :")
    print(target_point_1)
    
    target_point_2 = {tonumber(s[3]), tonumber(s[4])}
    print("target_point_2 :")
    print(target_point_2)
    
    target_point_3 = {tonumber(s[5]), tonumber(s[6])}
    print("target_point_3 :")
    print(target_point_3)
    
    target_point_4 = {tonumber(s[7]), tonumber(s[8])}
    print("target_point_4 :")
    print(target_point_4)
    
end


function mysplit (inputstr, sep)
        if sep == nil then
                sep = "%s"
        end
        local t={}
        for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
                table.insert(t, str)
                --print(str)
        end
        return t
end


function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
