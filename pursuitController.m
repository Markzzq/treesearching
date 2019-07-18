% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019


function [Gamma, number_state] = pursuitController(p,Ld,path,number_state)

global L

pathlength = length(path); % calculate the length of path matrix

% generate a short path matrix avoiding too much calculation
if number_state+100 <= pathlength
    path = path(:,number_state:number_state+100);
elseif number_state+100 > pathlength
    if number_state < pathlength
    path = path(:,number_state:pathlength);
    elseif number_state >= pathlength
        path = path(:,pathlength);
    end
end

gn = length(path); % gn is the length of the new path matrix

sigma_l = abs(Ld - ((p(1,1)- path(1,1))^2 + (p(2,1) - path(2,1))^2)^(0.5));
sigma_e = ((p(1,1)- path(1,1))^2 + (p(2,1) - path(2,1))^2)^(0.5);
pathgoal = zeros(2,1);
Errorpoint = zeros(2,1);
% arbitrary choose the difference and error initial value (waits to be updated)
for g = 1:gn-1
    
    % to find the theta, each sintheta value, and the temp distance between
    % each point to robot state point
    goal_temp = norm([path(1,g) path(2,g)] - [p(1,1) p(2,1)]);
    theta_g = acos( (path(1,g)-p(1,1)) / goal_temp);
    sintheta_g =  (path(2,g)-p(2,1)) / goal_temp;
    if sintheta_g <= 0  % to make sure the real theta value
        theta_g = 2 * pi - theta_g;
    end
    
    % if the path point is in front of the robot, then action
    if cos( theta_g - p(3,1) ) >= 0 
        
            Ld_temp = ((p(1,1)- path(1,g))^2 + (p(2,1) - path(2,g))^2)^(0.5);
            dif = abs(Ld - Ld_temp);
            if dif <= sigma_l
                sigma_l = dif;  % update the difference value 
                pathgoal(1,1) = path(1,g);  % put this goal into pathgoal matrix
                pathgoal(2,1) = path(2,g);
            end
       
    end 
    % update the error value 
    error = ((p(1,1)- path(1,g))^2 + (p(2,1) - path(2,g))^2)^(0.5);
    if error <= sigma_e
        sigma_e = error;
        Error = error;
        number_state = number_state + g-1;
    end
    
end

    % when the goal point is selected, calculate it again
    goal_temp = norm([pathgoal(1,1) pathgoal(2,1)] - [p(1,1) p(2,1)]);
    theta_g = acos( (pathgoal(1,1)-p(1,1)) / goal_temp);
    sintheta_g = (pathgoal(2,1)-p(2,1)) / goal_temp;
    if sintheta_g <= 0
        theta_g = 2 * pi - theta_g;
    end
    % calculate the K and Gamma as output for this function
    error_y = goal_temp * sin( theta_g - p(3,1) );
    K = 2 * error_y / (Ld * Ld);
    Gamma = atan( K * L);
    
    


