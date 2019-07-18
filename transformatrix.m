% EBS 289K homework #1
% author  Ziqian Zhu / Yufei Wang
% date  04/15/2019

% this function is to calculate the transfer matrix of the new position
function T = transformatrix (x, y, theta)
T = transl2(x, y) * trot2(theta * 180 / pi, 'deg');