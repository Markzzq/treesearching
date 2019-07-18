% test image processing

%% clear all previous data
clear all
close all
clc


%% generate the bitmap of the Nursery and relative parameters
global bitmap Xmax Ymax
global C R K W
generateNursery;

C = 5000;
R = 5000;
% model parameters:
global L wid  tau_gamma tau_v gamma_max Vmax Ld RL
global bitmap_new occ_grid

L = 3;  % L is the Wheelbase of the robot
wid = 2;  % W is the width of the robot
tau_gamma = 0.1;  % tau_gamma is the steering lag
tau_v = 0.2;  % tau_v is the velocity lag
gamma_max = 55*pi/180;  % gamma_max is the maximum steering angle 
Vmax = 1;  % Vmax is the maximum velocity
Ld = 5;  % Ld is the detection distance in the pursuitContoller 
RL = 20;   % RL is the length of each row 
W = 3;  % W is the width of the forest



% other initials  
global  DT dT GT N 

T = 100;  % T is the operation time 
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



%%
    % call the function 2
    ll = length(path);
    bitmap = flipud(bitmap);  %   ?????
    
    for k = 1:ll
        
    noise = 0.2*randn(1);
%     noise = 0;
    
    Tl = SE2([path(1,k) path(2,k) pi/2+noise]);   % Tl stors the local coodinate of the robot when using laser  /// Tl is same as q_true
    Tl = SE2([10 10 pi/2+noise]);   % Tl stors the local coodinate of the robot when using laser  /// Tl is same as q_true
    
    
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

    % low pass filter
%     j = 1;
%     while j <= h
%         o = 1;
%         while isinf(p(j,2)) == 0
%             o = o + 1;
%             j = j + 1;
%             if j > 1440
%                 break
%             end
%             
%         end
%         p(j-o+1:j,2) = filter (0.1, [1 -0.1], p(j-o+1:j,2));
%         
%     end
        

    

    
    
% update the Occupacy_grid
    
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % handle infinite range0
        if(isinf(range)) 
            range = rangeMax+1;
        end
        
        % the pose is what the robot think it will be 
        Tl = SE2([path(1,k) path(2,k) pi/2+noise]);
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax);
%         n = updateLaserBeamBitmap(angle, range, Tl.T, R, C, Xmax, Ymax);
    end
    
%     figure(2)
% %     plotvol([0 50 0 50]);
%     hold on 
%     plot(path(1,k),path(2,k),'.','color','r')
%     pause(0.01);
%     plot(path(1,k),path(2,k),'.','color','w')
%     imagesc(bitmap_new);
    
    
    end

    
    
    