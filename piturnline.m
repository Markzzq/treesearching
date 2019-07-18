% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019

% this function is to generate the path of pi shape path
function h = piturnline(i,newnode)

global r_min 

x1 = newnode(1,i);
y1 = newnode(2,i);

x2 = newnode(1,i+1);
y2 = newnode(2,i+1);

a = 0:0.05:(pi/2);
b = 0:0.05:(abs(x2-x1)-(2*r_min));

if y1 > 20 
    if x1 < x2
        r1 = x1 + r_min;
        r2 = x2 - r_min;
        h(1,:) = r1 + r_min * sin(a - pi/2);
        h(2,:) = y1 + r_min * cos(a - pi/2);
        h1(1,:) = r1 + b;
        h1(2,:) = y1 + r_min;
        h2(1,:) = r2 + r_min * sin(a);
        h2(2,:) = y2 + r_min * cos(a);
        h =[h h1 h2];
    elseif x1 > x2
        r1 = x1 - r_min;
        r2 = x2 + r_min;
        h(1,:) = r1 + r_min * sin(a + pi/2);
        h(2,:) = y1 + r_min * cos(a - pi/2);
        h1(1,:) = r1 - b;
        h1(2,:) = y1 + r_min;
        h2(1,:) = r2 - r_min * sin(a);
        h2(2,:) = y2 + r_min * cos(a);
        h = [h h1 h2];
    end
    
    
elseif y1 == 20
    if x1 < x2 
        r1 = x1 + r_min;
        r2 = x2 - r_min;
        h(1,:) = r1 + r_min * sin(a - pi/2);
        h(2,:) = y1 - r_min * cos(a - pi/2);
        h1(1,:) = r1 + b;
        h1(2,:) = y1 - r_min;
        h2(1,:) = r2 + r_min * sin(a);
        h2(2,:) = y2 - r_min * cos(a);
        h = [h h1 h2];
    elseif x1 > x2
        r1 = x1 - r_min;
        r2 = x2 + r_min;
        h(1,:) = r1 + r_min * sin(a + pi/2);
        h(2,:) = y1 - r_min * cos(a - pi/2);
        h1(1,:) = r1 - b;
        h1(2,:) = y1 - r_min;
        h2(1,:) = r2 - r_min * sin(a);
        h2(2,:) = y2 - r_min * cos(a);
        h = [h h1 h2];
    end
end
