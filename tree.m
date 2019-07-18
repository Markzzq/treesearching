
% and find the trees

close all
clear all


% load testimage1.mat
load finalbitmap_2.mat


global bitmap_new C R Xmax Ymax

bitmap_new = bitmap_new_zhu;

R = 5000;
C = 5000;
Xmax = 50;
Ymax = 50;

Cy = 0.9 * R;
Rx = 0.7 * C;

i = 20;
j = 20;
k = 1;

left = zeros(60,2);
right = zeros(60,2);

while j <= Cy
    i = 3;

    while i <= Rx
        u = 1;
        if bitmap_new(i,j) > 1e-11
            while bitmap_new(i,j) > 1e-50
                i = i + 1;
                u = u + 1;
            end
        
            if rem(u,2) == 0
                i = i - (u/2);
                    if bitmap_new(i,j-1) < 1e-180
                        left(k,1) = i;
                        left(k,2) = j;

                        n = 1;
                        while bitmap_new(i,j+n) > 0
                        n = n + 1; 
                            if j + n > Cy 
                                break
                            end
                        end
                        right(k,1) = i;
                        right(k,2) = j + n - 1;
                        k = k + 1;
                    end 

            elseif rem(u,2) == 1
                i = i - (u-1)/2;
                    if bitmap_new(i,j-1) < 1e-180
                        left(k,1) = i;
                        left(k,2) = j;

                        n = 1;
                        while bitmap_new(i,j+n) > 0
                        n = n + 1; 
                            if j + n > Cy 
                                break
                            end
                        end
                        right(k,1) = i;
                        right(k,2) = j + n - 1;
                        k = k + 1;
                    end
            end
        
        end
        i = i + u;
    end
    j = j + 1;
    
end
        

% calculate the middle of the tree

l = length(left);
for t = 1:l
    middle(t,1) = ceil((left(t,1) + right(t,1))/2);
    middle(t,2) = ceil((left(t,2) + right(t,2))/2);
    
    radius(t,1) = (right(t,2) - left(t,2))/2;
    
    pose(t,1) = middle(t,2) * Ymax / R;
    pose(t,2) = Ymax - middle(t,1) * Xmax / C;
    pose(t,3) = radius(t,1) * Xmax / R;
    
end


% output the file

line1 = [];
line2 = [];
line3 = [];
line4 = [];
line5 = [];

for t = 1:l
    if (17 < pose(t,1) && pose(t,1) <= 20) == 1
        line1 = [line1;pose(t,:)];
    elseif (20 < pose(t,1) && pose(t,1) <= 23) == 1
        line2 = [line2;pose(t,:)];
    elseif (23 < pose(t,1) && pose(t,1) <= 26) == 1
        line3 = [line3;pose(t,:)];
    elseif (26 < pose(t,1) && pose(t,1) <= 29) == 1
        line4 = [line4;pose(t,:)];
    elseif (29 < pose(t,1) && pose(t,1) <= 32) == 1
        line5 = [line5;pose(t,:)];
    end
end






load originalbitmap.mat


global bitmap_zhu 



R = 5000;
C = 5000;
Xmax = 50;
Ymax = 50;

Cy = 0.9 * R;
Rx = 0.7 * C;

i = 20;
j = 20;
k = 1;

orileft = zeros(60,2);
oriright = zeros(60,2);

while j <= Cy
    i = 3;

    while i <= Rx
        u = 1;
        if bitmap_zhu(i,j) > 1e-11
            while bitmap_zhu(i,j) > 1e-50
                i = i + 1;
                u = u + 1;
            end
        
            if rem(u,2) == 0
                i = i - (u/2);
                    if bitmap_zhu(i,j-1) < 1e-180
                        orileft(k,1) = i;
                        orileft(k,2) = j;

                        n = 1;
                        while bitmap_zhu(i,j+n) > 0
                        n = n + 1; 
                            if j + n > Cy 
                                break
                            end
                        end
                        oriright(k,1) = i;
                        oriright(k,2) = j + n - 1;
                        k = k + 1;
                    end 

            elseif rem(u,2) == 1
                i = i - (u-1)/2;
                    if bitmap_zhu(i,j-1) < 1e-180
                        orileft(k,1) = i;
                        orileft(k,2) = j;

                        n = 1;
                        while bitmap_zhu(i,j+n) > 0
                        n = n + 1; 
                            if j + n > Cy 
                                break
                            end
                        end
                        oriright(k,1) = i;
                        oriright(k,2) = j + n - 1;
                        k = k + 1;
                    end
            end
        
        end
        i = i + u;
    end
    j = j + 1;
    
end
        

% calculate the middle of the tree

l = length(orileft);
for t = 1:l
    orimiddle(t,1) = ceil((orileft(t,1) + oriright(t,1))/2);
    orimiddle(t,2) = ceil((orileft(t,2) + oriright(t,2))/2);
    
    oriradius(t,1) = (oriright(t,2) - orileft(t,2))/2;
    
    oripose(t,1) = orimiddle(t,2) * Ymax / R;
    oripose(t,2) = Ymax - orimiddle(t,1) * Xmax / C;
    oripose(t,3) = oriradius(t,1) * Xmax / R;
    
end


% output the file

oriline1 = [];
oriline2 = [];
oriline3 = [];
oriline4 = [];
oriline5 = [];

for t = 1:l
    if (17 < oripose(t,1) && oripose(t,1) <= 20) == 1
        oriline1 = [oriline1;oripose(t,:)];
    elseif (20 < oripose(t,1) && oripose(t,1) <= 23) == 1
        oriline2 = [oriline2;oripose(t,:)];
    elseif (23 < oripose(t,1) && oripose(t,1) <= 26) == 1
        oriline3 = [oriline3;oripose(t,:)];
    elseif (26 < oripose(t,1) && oripose(t,1) <= 29) == 1
        oriline4 = [oriline4;oripose(t,:)];
    elseif (29 < oripose(t,1) && oripose(t,1) <= 32) == 1
        oriline5 = [oriline5;oripose(t,:)];
    end
end

error = [0.04 0.01;
         0.023 0.01;
         0.023 0.01;
         0.014 0.01;
         0.023 0.01;
         0.014 0.01;
         0.023 0.01;
         0.023 0.01;
         0.023 0.01;
         0.014 0.02;
         0.023 0;
         0.031 0;
         0.031 0;
         0.023 0;
         0.023 0;
         0.031 0;
         0.023 0.01;
         0.023 0.005;
         0.028 0.01;
         0.051 0.01;
         0.014 0.01;
         0.041 0.01;
         0.023 0.01;
         0.023 0.01;
         0.041 0.005;
         0.023 0.01;
         0.023 0;
         0.02 0;
         0.01 0;
         0.02 0;
         0.02 0;
         0.01 0.005;
         0.014 0.005;
         0.014 0.03;
         0.045 0.015;
         0.036 0;
         0.023 0.015;
         0.042 0.005;
         0.036 0.03;
         0.05 0.01;
         0.023 0.005;
         0.023 0.005;
         0.041 0.005];
     
     
     

