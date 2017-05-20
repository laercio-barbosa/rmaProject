function [clientID, vrep] = initVrep()
%INITVREP Summary of this function goes here

    vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
end

