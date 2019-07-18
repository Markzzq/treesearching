% EBS 289K homework #4
% author  Ziqian Zhu 
% date  04/25/2019

% this function is to calculate the new positon of the tractor based on the
% original tractor matrix and the tranformatrix of that new point.
% And plot the figure of robot and its motion and trace 
function  plotTractor(Tractor,p)

h = 1;  % count of the time steps
global L 

T = transformatrix(p(1,h), p(2,h), p(3,h));  % Transer matrix for rear wheel
% TB = transformatrix(p(1,h) + L * cos(p(3,h)), p(2,h) + L * sin(p(3,h)), p(3,h) + p(5,h));  % transfer matrix for front wheel

Tractor1 = T * Tractor;  % rear wheel
% Tractor2 = TB * Tractor;  % front wheel


for h = 2:length(p)

% plot(pathgoal(1,h-1),pathgoal(2,h-1),'*','color','red');
% pause(0.1)
% plot(pathgoal(1,h-1),pathgoal(2,h-1),'*','color','white');
    
% eliminate the old figure
plot([Tractor1(1,1:4),Tractor1(1,1)],[Tractor1(2,1:4),Tractor1(2,1)],'color','white');
% plot([Tractor1(1,5),Tractor2(1,5)],[Tractor1(2,5),Tractor2(2,5)],'color','white');
% plot([Tractor2(1,1:4),Tractor2(1,1)],[Tractor2(2,1:4),Tractor2(2,1)],'color','white');

% update the new transfer matrix and tractors
T = transformatrix(p(1,h), p(2,h), p(3,h));
% TB = transformatrix(p(1,h) + L * cos(p(3,h)), p(2,h) + L * sin(p(3,h)), p(3,h) + p(5,h));

Tractor1 = T * Tractor;
% Tractor2 = TB * Tractor;

% plot the new figure
plot([Tractor1(1,1:4),Tractor1(1,1)],[Tractor1(2,1:4),Tractor1(2,1)],'color','red');
% plot([Tractor1(1,5),Tractor2(1,5)],[Tractor1(2,5),Tractor2(2,5)],'color','red');
% plot([Tractor2(1,1:4),Tractor2(1,1)],[Tractor2(2,1:4),Tractor2(2,1)],'color','red');

% plot the trace 
% plot(p(1,h), p(2,h),'.','color','red');  % plot trace of robot's frame origin
pause(0.05);

end


