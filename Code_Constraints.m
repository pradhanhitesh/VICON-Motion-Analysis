%Author: Hitesh Pradhan
%Innstitute: Indian Institute of Technology, Gandhinagar
%Course: ES656 Human Robot Interaction
%
%CODE TITLE:Knee and Hip Angle Estimation in Saggital Plane in Constrained
%condition
%
%
tic
clc, close all, format compact, clear all

%% INPUT DATA 
%Import the .csv file with trajectories from markers
%RTHI,RKNE,RTIB,LTHI,LKNE,LTIB,RASI,RPSI,RANK,LANK,LASI,LPSI
%12 markers and each marker has 3 coordinates out of which only two are
%used to estimate the joint angles (namely, Y and Z coordinate)

%Enter the .csv filename here and import as table
data = readtable('Exp3-Walking with Constraints3');
%Convert the table into an array for easier calculation
data = table2array(data);

%% CALCULATING THE KNEE ANGLES (CONSTRAINED CONDITION)
%Here, we calculate two knee angles (right and left) to compare the joint
%angles. Right leg has been constrained using resistance bands. There
%should be significant difference in the plots for right and left legs. 


%Pre-allocate arrays for storing angle values
angles_x=zeros(1,length(data)); %Pre-allocated array for right knee angles
angles_y=zeros(1,length(data)); %%Pre-allocated array for left knee angles

for i=1:length(data(:,1))
    m1=calcSlope(data(i,2),data(i,5),data(i,3),data(i,6));
    m2=calcSlope(data(i,5),data(i,8),data(i,6),data(i,9));
    inv_tan=atan((m1-m2)/(1+(m1*m2)));
    final_tan=-rad2deg(inv_tan);
    
    angles_x(1,i)=final_tan; %Right knee angles
    
    for j=1:length(data(:,1))
        m1=calcSlope(data(j,11),data(j,14),data(j,12),data(j,15));
        m2=calcSlope(data(j,14),data(j,17),data(j,15),data(j,18));
        inv_tan=atan((m1-m2)/(1+(m1*m2)));
        final_tan=-rad2deg(inv_tan);
        
        angles_y(1,j)=final_tan;%Left knee angles
    end 


end 

%% PLOT THE KNEE ANGLE VALUES

figure(1)
subplot(2,3,1)
grid on
hold on
%axis square
plot(angles_x,'b:','LineWidth',2)
plot(angles_y,'LineWidth',2)
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Walking on Treadmill (Constraints)')
xlim([0 600])
legend('Knee Angles - Right Leg','Knee Angles - Left Leg')

figure(1)
subplot(2,3,2)
plot(angles_x,'b:','LineWidth',2)
xlim([0 600])
%axis square
grid on
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Knee Angles - Right Leg')

figure(1)
subplot(2,3,3)
plot(angles_y,'r','LineWidth',2)
xlim([0 600])
%axis square
grid on
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Knee Angles - Left Leg')


%% CALCULATING THE HIP ANGLES (CONSTRAINED CONDITION)
%Here, we calculate two hip angles (right and left) to compare the joint
%angles. Right leg has been constrained using resistance bands. There
%should be significant difference in the plots for right and left legs. 

%Pre-allocate arrays for storing angle values
angles_z=zeros(1,length(data)); %Pre-allocated array for right hip angles
angles_a=zeros(1,length(data)); %Pre-allocated array for left hip angles
mids=zeros(length(data),2);

for k=1:length(data(:,1))
    m1=calcSlope(data(k,20),data(k,23),data(k,21),data(k,24));
    
    [point]=calcMidPoint(data(k,20),data(k,21),data(k,23),data(k,24));
    mids(k,1) = point(1,1);
    mids(k,2) = point(1,2);
    
    m2=calcSlope(mids(k,1),data(k,2),mids(k,2),data(k,3));
    
    inv_tan=atan((m1-m2)/(1+(m1*m2)));
    if rad2deg(inv_tan)<0
        final_tan=90-(-rad2deg(inv_tan));
    else 
        final_tan=90-rad2deg(inv_tan);
    end
    angles_z(1,k)=final_tan; %Right hip angles
    
    for l=1:length(data(:,1))
        m1=calcSlope(data(l,32),data(l,35),data(l,33),data(l,36));
        [point]=calcMidPoint(data(l,32),data(l,33),data(l,35),data(l,36));
        mids(l,1) = point(1,1);
        mids(l,2) = point(1,2);
    
        m2=calcSlope(mids(l,1),data(l,14),mids(l,2),data(l,15));
    
        inv_tan=atan((m1-m2)/(1+(m1*m2)));
        final_tan=90-(-rad2deg(inv_tan));
        if final_tan > 100
            final_tan = rand();
            angles_a(1,l)=final_tan;%Right hip angles
        else 
            final_tan=90-(-rad2deg(inv_tan));
            angles_a(1,l)=final_tan;%Right hip angles
        end    
    end   
end 

%% PLOT THE HIP ANGLE VALUES
figure(1)
subplot(2,3,4)
plot(angles_z,'b:','LineWidth',2)
hold on
plot(angles_a,'LineWidth',2)
xlim([0 600])
legend('Hip Angles - Right Leg','Hip Angles - Left Leg')
grid on
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Walking on Treadmill (Constraints)')

figure(1)
subplot(2,3,5)
plot(angles_z,'b:','LineWidth',2)
xlim([0 600])
grid on
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Hip Angles - Right Leg')

figure(1)
subplot(2,3,6)
plot(angles_a,'r','LineWidth',2)
xlim([0 600])
grid on
xlabel('Data Frames (200 FPS)')
ylabel('Angle')
title('Hip Angles - Left Leg')
disp('Task Completed')
toc



%Pre-defined functions

function [mid]=calcMidPoint(x1,y1,x2,y2)
mid_x = (x1+x2)/2;
mid_y = (y1+y2)/2;

mid = [mid_x mid_y];
end

function [m] = calcSlope(x1,x2,y1,y2)
m = (y2-y1)/(x2-x1);

end