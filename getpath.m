% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019

function p = getpath(route,node,W,K)

global r_min RL

% generate three parts of the total path including start point to row
% points, row path, and the path from row points to end point
for i = 1:2*K+4
    j = route(1,i);
    newnode(1,i) = node(1,j);
    newnode(2,i) = node(2,j);
end

% generate a path from (0,0) to (17,30)
 xc = linspace(0,20-W,300);
 yc = linspace(0,20,300);
 h(1,1:300) = xc;
 h(2,1:300) = yc;
 oldpath = h;

% i = 1;
% 
% path = startline(i,newnode); % oldpath is calculated through above
% oldpath = [oldpath path];
% 
% newnode(1,1) = 0;
% newnode(2,1) = 0;
% 
% newnode = [0 20-W 20-W 20 20 20+W 20+W 20+2*W 20+2*W 20+3*W 20+3*W 20+(K-2)*W 20+(K-2)*W ]





for i = 1:(2*K+3)
    
    if abs(route(1,i+1) - route(1,i)) == (K+2)
        path = straightline(i,newnode);
    elseif abs(route(1,i+1) - route(1,i)) * W > 2*r_min
        path = piturnline(i,newnode);
    elseif abs(route(1,i+1) - route(1,i)) * W <= 2*r_min
        path = omegaturnline(i,newnode);
    end
    
    path = [oldpath path];
    oldpath = path;
end

% i = 2*K+1;
% 
% pathfinal = endline(i,newnode); % path is calculated through above
% path = [oldpath pathfinal];

p = path;






