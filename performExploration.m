function [vLeft, vRight] = performExploration(clientID, vrep, sensorHandle)
%PERFORMEXPLORATION Summary of this function goes here
%   Detailed explanation goes here

    % Scene and pioneer motion variables
    noDetectionDist  = 1.0;  % No  detection distance
    maxDetectionDist = 0.05; % Max detection distance
    detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    v0 = 2.0;

    for i = 1:16
        [erro, state, coord, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_buffer);
        if (erro == vrep.simx_return_ok)
            dist = sqrt(power(coord(1),2) + power(coord(2),2) + power(coord(3),2));
            if (state == true && dist < noDetectionDist)
                if (dist < maxDetectionDist)
                    dist = maxDetectionDist;
                end
                detect(i) = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist));
            else
                detect(i) = 0;
            end
        else
            detect(i) = 0;
        end
    end

    vLeft  = v0;
    vRight = v0;

    for i = 1:16
        vLeft  = vLeft  + braitenbergL(i) * detect(i);
        vRight = vRight + braitenbergR(i) * detect(i);
    end
end

