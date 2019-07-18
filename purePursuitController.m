function [angle, error, index] = purePursuitController(q, L, Ld, path, start_index)
    % q -> state vector, 5-by-1 vector
    % L -> wheelbase
    % Ld -> predefined distance
    % path -> array of target positions
    % angle -> steering angle control
    % error -> minimum distance to the target points
    
    % Determine the current location of the vehicle
    current_x = q(1);
    current_y = q(2);
    % Find the path point closest to the vehicle
    [error, index] = findMinimumDistance(current_x, current_y, path, start_index);
    
    % Find the goal point
    if (error > Ld) % Ld is too small or start to find the first target point
        goal = path(:, index);
    else
        flag = index;
%         if (index < size(path, 2) - 100)
%             limit = index + 100;
%         else
%             limit = size(path, 2);
%         end
        for i = index:size(path, 2)
            dis = (current_x-path(1,i))^2 + (current_y-path(2,i))^2 - Ld;
            if (dis >= 0)
                flag = i - 1;
                break;
            end
        end
        goal = path(:, flag);
    end
    
    % Transform to the vehicle coordinate
    goal_v = trot2(-q(3)) * [goal(1)-current_x; goal(2)-current_y; 1];
    
    % Calculate the curvature
    ey = goal_v(2);
    angle = atan(2*ey*L/(Ld*Ld));
end