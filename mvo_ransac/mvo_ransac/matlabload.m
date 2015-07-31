%%This file is for Monocular Visual Odometry algorithm visualization
clear all; clc; close all;


%load  the result of Monocular Visual Odometry
fs = fopen('data.dat', 'rb');
db = fread(fs, 'double'); 
fclose(fs);
N=length(db);
n=N/16;
out_cal = reshape(db,16,n); 

p1=eye(4);
t=[];

data=load('00.txt');
t1=[];

%load the ground true trajectory data
for i=2:n

	out1=reshape(out_cal(:,i), 4, 4);
	out1=out1';
	p1=p1*inv(out1);
	t(i,:)=[p1(1,4),p1(3,4)];%the computational pose of camera
	
	d1=reshape(data(i,:),4,3);
    d1=d1';
    temp=[0,0,0,1];
    d=[d1;temp];
   t1(i,:)=[d(1,4),d(3,4)];%the  true pose of camera
   
end

plot(t(:,1),t(:,2),'--',t1(:,1),t1(:,2),'r');

title('定位结果比较');ylabel('Z(m)');xlabel('X(m)');
legend('使用RANSAC实验结果','真实轨迹');