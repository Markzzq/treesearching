% EBS 289K homework #
% author  Ziqian Zhu 
% date  04/25/2019

% this function is to create a serial matrix to describe the original
% position and angle of the robot 
function Tractor = tractor(robot_x_initial, robot_y_initial)
Tractor = zeros (3,5);  % set a zero matrix

% define the four corners of the tractor according to the parameter i
% decide in the main function
Tractor(1,1) = robot_x_initial + 1.25;
Tractor(2,1) = robot_y_initial + 1;

Tractor(1,2) = robot_x_initial + 1.25;
Tractor(2,2) = robot_y_initial - 1;

Tractor(1,3) = robot_x_initial - 1.25;
Tractor(2,3) = robot_y_initial - 1;

Tractor(1,4) = robot_x_initial - 1.25;
Tractor(2,4) = robot_y_initial + 1; 

Tractor(1,5) = robot_x_initial;
Tractor(2,5) = robot_y_initial;

Tractor(3,:) = 1;