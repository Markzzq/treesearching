function [distance, index] = findMinimumDistance(x, y, path, start_index)
    % x -> x coordinate of the current position
    % y -> y coordinate of the current position
    % path -> array of target points
    
    % Search for the closest node in the ranges
    if (start_index - 20 <= 0)
        start = 1;
    else
        start = start_index - 20;
    end
    
    if (start_index + 80 >= size(path, 2))
        final = size(path, 2);
    else
        final = start_index + 80;
    end
    
    
    % Initialize minimum distance and corresponding index
    min = Inf; index = 1;
    for i = start: final
        dis = (x-path(1,i))^2 + (y-path(2,i))^2;
        if(dis < min)
            min = dis;
            index = i;
        end
    end
    distance = min;
end