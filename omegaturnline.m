% EBS 289K homework #4
% author  Ziqian Zhu
% date  04/27/2019

% this function is to generate the path of omega path
function h = omegaturnline(i,newnode)

global r_min

x1 = newnode(1,i);
y1 = newnode(2,i);

x2 = newnode(1,i+1);
y2 = newnode(2,i+1);

x3 = (x1 + x2)/2;
gap = abs(x1 -x2);

l3 = sqrt((2*r_min)^(2) - (r_min + gap/2)^(2));

if y1 == 20
    y3 = y1 - l3;
elseif y1 ~= 20
    y3 = y1 + l3;
end

alpha = asin(l3/(2*r_min));
a = 0:0.05:alpha;
b = pi+alpha:-0.05:-alpha;
c = pi-alpha:0.05:pi;

if y1 > 20
    if x1 < x2
        r1 = x1 - r_min;
        r2 = x2 + r_min;
        r3 = x3;
        h(1,:) = r1 + r_min * cos(a);
        h(2,:) = y1 + r_min * sin(a);
        h1(1,:) = r3 + r_min * cos(b);
        h1(2,:) = y3 + r_min * sin(b);
        h2(1,:) = r2 + r_min * cos(c);
        h2(2,:) = y2 + r_min * sin(c);
        h = [h h1 h2];
    elseif x1 > x2
        r1 = x1 + r_min;
        r2 = x2 - r_min;
        r3 = x3;
        h(1,:) = r1 - r_min * cos(a);
        h(2,:) = y1 + r_min * sin(a);
        h1(1,:) = r3 - r_min * cos(b);
        h1(2,:) = y3 + r_min * sin(b);
        h2(1,:) = r2 - r_min * cos(c);
        h2(2,:) = y2 + r_min * sin(c);
        h = [h h1 h2];
    end
    
elseif y1 == 20 
    if x1 < x2
        r1 = x1 - r_min;
        r2 = x2 + r_min;
        r3 = x3;
        h(1,:) = r1 + r_min * cos(a);
        h(2,:) = y1 - r_min * sin(a);
        h1(1,:) = r3 + r_min * cos(b);
        h1(2,:) = y3 - r_min * sin(b);
        h2(1,:) = r2 + r_min * cos(c);
        h2(2,:) = y2 - r_min * sin(c);
        h = [h h1 h2];
    elseif x1 > x2
        r1 = x1 + r_min;
        r2 = x2 - r_min;
        r3 = x3;
        h(1,:) = r1 - r_min * cos(a);
        h(2,:) = y1 - r_min * sin(a);
        h1(1,:) = r3 - r_min * cos(b);
        h1(2,:) = y3 - r_min * sin(b);
        h2(1,:) = r2 - r_min * cos(c);
        h2(2,:) = y2 - r_min * sin(c);

        h = [h h1 h2];
    end
end



