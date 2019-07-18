% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019

% this function is to generate the path of a straight line
function h = straightline(i,newnode)

global RL

x1 = newnode(1,i);

y1 = newnode(2,i);

a = 0:0.1:RL;

n = length(a);


h(1,1:n) = x1;
if y1 == 20 
    h(2,1:n) =  a + y1;
elseif y1 ~= 20
    h(2,1:n) =  - a + y1;
end

