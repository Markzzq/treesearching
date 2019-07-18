% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019

function p = getnode(W,K,RL)

% define the global variables 
% global W K RL

p = zeros(2,2*K+4);  % genertat a zero matrix to store the xy coordinate

% p(1,1) = 0;  % get the initial node 1 
% p(2,1) = 0;
% 
% p(1,2*K+2) = 20 - 1.5*W;  % get the final node 2N+2
% p(2,2*K+2) = 20 + RL/2;

for i = 1:K+2
    p(1,i) = (i-2) * W + 20;  % get the lower hearland nodes 
    p(2,i) = 20;
    
    p(1,i+K+2) = (i-2) * W + 20;  % get the upper headland nodes
    p(2,i+K+2) = 20 + RL;
end
