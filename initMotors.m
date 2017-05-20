function [leftMotorHandle, rightMotorHandle] = initMotors(clientID, vrep, leftMotorDescr, rightMotorDescr)
%INITMOTORS Motors initialization

    [erro, leftMotorHandle] = vrep.simxGetObjectHandle(clientID, leftMotorDescr, vrep.simx_opmode_oneshot_wait);
    if (erro ~= 0)
        disp('Handle for left motor not found!');
    else
        disp('Connected to left motor!');
    end

    [erro, rightMotorHandle] = vrep.simxGetObjectHandle(clientID, rightMotorDescr, vrep.simx_opmode_oneshot_wait);
    if (erro ~= vrep.simx_return_ok)
        disp('Handle for right motor not found!');
    else
        disp('Connected to right motor!');
    end

end

