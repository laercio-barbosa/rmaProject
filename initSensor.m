function [sensorHandle] = initSensor(clientID, vrep)
%INITSENSOR Sensor initialization
    sensorHandle = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    for i = 1:16
        [erro, sensorHandle(i)] = vrep.simxGetObjectHandle(clientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],vrep.simx_opmode_oneshot_wait);
        if (erro ~= vrep.simx_return_ok)
            disp(['Handle for sensor Pioneer_p3dx_ultrasonicSensor' num2str(i) ' not found!']);
        else
            disp(['Connected to sensor Pioneer_p3dx_ultrasonicSensor' num2str(i) '!']);
            % We start streaming data, but ignore function outputs as
            % is the first call - see function help
            [~, ~, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_streaming);
        end
    end
end

