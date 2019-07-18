% EBS 289K Sensors and Acuators
% final project main function
% author: Ziqian Zhu
% date: 06/06/2019

%% clear all previous data
clear all
close all
clc


%% generate the bitmap of the Nursery and relative parameters
global bitmap Xmax Ymax
global C R K W
generateNursery;

bitmap = flipud(bitmap);

% model parameters:
global L wid  tau_gamma tau_v gamma_max Vmax Ld RL
global bitmap_new occ_grid

L = 3;  % L is the Wheelbase of the robot
wid = 2;  % W is the width of the robot
tau_gamma = 0.1;  % tau_gamma is the steering lag
tau_v = 0.2;  % tau_v is the velocity lag
gamma_max = 55*pi/180;  % gamma_max is the maximum steering angle 
Vmax = 1;  % Vmax is the maximum velocity
Ld = 2;  % Ld is the detection distance in the pursuitContoller 
RL = 20;   % RL is the length of each row 
W = 3;  % W is the width of the forest


% other initials  
global  DT dT GT N 

T = 150;  % T is the operation time 
K = 5;   % temp K = 5 because K defines the number of rows
DT = 0.01;
dT = 0.01;
GT = 1;   
N = T/DT;  % N is the number of the time intervals
bitmap_new = 0.5 * ones(R, C);  
occ_grid = 1 * ones(R, C);


% robot initial place and use it to generate the Tractor
initial_x = 0;  % those three reprsent the initial pose of the robot
initial_y = 0;
initial_theta = pi/2;
Tractor = tractor(initial_x, initial_y);  % build the tractor


% robot initial state 
q(1) = 0;
q(2) = 0;
q(3) = pi/2;
q(4) = 0;
q(5) = 0;

q_true = q;
q_predict = q;
P1_final = zeros(3,3);


%% function input

% fucntion 1 

% Control constraints
umax = [gamma_max Vmax]';
umin = - umax;
% state constraints
Qmax(1) = Inf; Qmax(2) = Inf; Qmax(3) = Inf;
Qmax(4) = gamma_max; Qmax(5) = Vmax;
Qmin = -Qmax;


% function 2

rangeMax = 20;  % rangeMax is 20 meters 
angleSpan = pi;   % degree or rad ??????  perheps rad 
angleStep = angleSpan/1440;    % what is the angular resolution mean  180/0.125 ????
% Xmax = 50;  % those two value related to how many rows we have K  ??????
% Ymax = 50;
% R = Xmax*100;
% C = Ymax*100;
% map = zeros(R,C);

errors = zeros(1, N);
index = 1;




%% path Planner 
path = pathPlanner(K,W);
number_state(1,1) = 1;  % this number is to calculate the number of the nearest node




%% EKF 
for k = 1:N 
    
    % path controller will output the   Vd and Wd  to the robot model based
    % on the location of the robot, which is the state q_predict(x,y,theta)
    % from EKF 
    
    
    % path controller output the V_d and Gamma_d 
%     [gamma, error, goal] = pursuitController(q_true',Ld,path);
%     Error(1,k) = error;
%     pathgoal(:,k) = goal;
    
%     [gamma,velocity] = pursuitController(q_predict',Ld,path);  % my one 
    [gamma,number_state] = pursuitController(q_predict',Ld,path,number_state(1,k));
    number_state(1,k+1) = number_state;
    
%     [gamma, errors(k), index] = purePursuitController(q_predict', L, Ld, path, index);  %  ??? 
%     if (index == size(path, 2))
%         break;
%     end
    
    
    
    % call function 1 
    % input 
    u(1) = gamma;
    u(2) = Vmax;
    [q_true_next, odo] = robot_odo(q_true, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v);
    
    
    % every time the prediction need to be done 
    
    % use the  odo  from the function 1 to do the prediction 
    
    q_predict(1) = q_predict(1) + odo(1)*cos(q_true(3));
    q_predict(2) = q_predict(2) + odo(1)*sin(q_true(3));
    q_predict(3) = q_predict(3) + odo(2);
    q_predict(4) = q_predict(4);
    q_predict(5) = q_predict(5);
    
    if  rem(k,10) == 0  % measurement update need to be use to update the state of the robot
    
    % the subtitle  1   means this is the first error source from odometry
    % use q_true_next and q_predict to calculate the Q1 covariance of model
    % Q1 matrix should be 2 by 2   P1_pre is 3 by 3 matrix
    KA = [1 0 -odo(1)*sin(q_true(3));
         0 1 odo(1)*cos(q_true(3));
         0 0 1];
    KE = [cos(q_true(3)) 0;
         sin(q_true(3)) 0;
          0 1];
    
    Q1 = [0.012 0; 0 0.091];
    P1_pre = KA*P1_final*KA' + KE*Q1*KE';      % P1_pre is the predictive covariance from step 1 /// P1_final is the final covariance from previous step // Q1 is the model covariance
    % A is 3 by 3 matrix  and E is 3 by 2 matrix   ???? how to add them together  
    P1_final = P1_pre;
    

        % the subtile 2   means that this is hte second error source from GPS
        % input location is from q_true  returns the noisy measuremnt of the pose of the robot /// angle is rad  
        [x_n, y_n, theta_n] = GPS_CompassNoisy(q_true_next(1), q_true_next(2), q_true_next(3));
    
        % use x_n y_n and theta_n and q_true to calculate the Covariance R2 of
        % measurement   // R2 should be a 3 by 3 matrix 
        % C is 1 by 3 matrix  and F is also 3 by 3 matrix 
        e = [x_n; y_n; theta_n] - [q_predict(1); q_predict(2); q_predict(3)];
        KC = eye(3);
        KF = eye(3);
        
        R2 = [0.09036e-3 0 0; 0 0.08778e-3 0; 0 0 0.4003e-3];
        Lk = P1_pre*KC'/(KC*P1_pre*KC' + KF*R2*KF');
        q_update = q_predict + [(Lk * e)' 0 0];
        P2_update = P1_pre - Lk*KC*P1_pre;
        P1_final = P2_update; 
        
        q_predict = q_update;
        
    end
    
   
    
    % after EKF the new state will be   q_predict into the path controller
    % to get the next control input
    

%         q_realpose(k,:) = q_true_next;
        q_thinkitpose(k,:) = q_predict;
        q_true = q_true_next;
        
        
   
    
    % use lidar to detect the trees and update the occ_grid 
    % the pose as the input of the laser scanner is q_ture, and the filter
    % need to be used to filter the noise 
    
    % call the function 2
    
    Tl = SE2([q_true(1) q_true(2) q_true(3)]);   % Tl stors the local coodinate of the robot when using laser  /// Tl is same as q_true
    
    
    p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmap, Xmax, Ymax);
%     p(isinf(p(:,2)),2)= 0;  % not include the inf number 
    
    % this function is used to get the angle of the point and its range 
    % the angle is accurate and the range has noise ????
    
    % p(:,1) denots the ray angle
    % p(:,2) denots the range noise
    
    % next step is to use a filter to sliminate the range noise and then
    % update the occupacygrid 
    
    
    % median filter
    
    h = length(p);
    for j = 1:5
        p(j,2) = median(p(1:6+j,2));
    end
    for j = 6:h-5
        p(j,2) = median(p(j-5:j+5,2));
    end
    for j = h-4:h
        p(j,2) = median(p(j-5:h,2));
    end



%     % plot the vehicle 
%     figure(2)
%     plotvol([0 50 0 50]);
%     hold on;
% %     plot(path(1,1:2300),path(2,1:2300),'color','b');
% %     pause(0.01);
%     if k >1
%     plot([Tractor1(1,1:4),Tractor1(1,1)],[Tractor1(2,1:4),Tractor1(2,1)],'color','white');
%     end
% %     T = transformatrix(q_true(1), q_true(2), q_true(3));
%     T = transformatrix(q_thinkitpose(k,1), q_thinkitpose(k,2), q_thinkitpose(k,3));
% %     T = transformatrix(q_realpose(k,1), q_realpose(k,2), q_realpose(k,3));
%     Tractor1 = T * Tractor;  % rear wheel
%     plot([Tractor1(1,1:4),Tractor1(1,1)],[Tractor1(2,1:4),Tractor1(2,1)],'color','red');
     
    
    
    
    % update the Occupacy_grid
    
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % handle infinite range
        if(isinf(range)) 
            range = rangeMax+1;
        end
        
        % the pose is what the robot think it will be 
%         Tl = SE2([q_predict(1) q_predict(2) q_predict(3)]);
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax);
    end
    
    
%     figure(3)
%     imagesc(bitmap_new);
    
    
    
    
end
    
    
    
    




































