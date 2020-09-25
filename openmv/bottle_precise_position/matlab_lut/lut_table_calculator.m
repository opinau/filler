% matlab code for building LUT
% please remove camere distortions first!
% lut_distance contains pixel laser line location vs distance

clear all

fileID = fopen('lut_table_1.txt','w');
%% laser line pos - distance LUT
% code is using polynomal matching with 6th order 
%please enter value in mm in first column, pixel location in second
A= [
    102 2
    105 7.5
    110 10; 
    120 22; 
    130 30;
    140 37;
    150 44;
    160 50;
    170 55 
    180 59.5;
    190 63; 
    200 66; 
    210 69.5
    220 72
    230 74
    250 79
    260 81
    270 82
    300 87
    350 92
    400 95 ]
    
    
plot(A(:,2),A(:,1))

f=fit(A(:,2),A(:,1),'poly6')
x=1:101
lot_elementi= f.p1*x.^6 + f.p2*x.^5 + f.p3*x.^4+  + f.p4*x.^3+  + f.p5*x.^2+ f.p6*x + f.p7
index=1:length(x)

hold on
plot(x,lot_elementi,'r')


fprintf(fileID,'lut_distance = {');

for i=index
    fprintf(fileID,'%2.1d:',x(i));
    fprintf(fileID,' %2.1d,\n',round(lot_elementi(i)));
end
fprintf(fileID,'}');

fclose(fileID)
