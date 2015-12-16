  clc;clear all;close all;
data=load('pose.txt');%代码运算pose值
dt=load('00.txt');%地面真值
n=size(data,1)
translationError=[];
rotationError=[];
 temp=[0,0,0,1];
dist=0;
p0=eye(4);
length=[];
for i=1:n

    d1=reshape(data(i,:),4,3);
    d1=d1';
    d=[d1;temp];%计算的结果
%     p0=p0*inv(d);
%        t1(i,:)=[p0(1,4),p0(3,4)];
  t1(i,:)=[d(1,4),d(3,4)];    
 
    dt0=reshape(dt(i+1,:),4,3);
    dt0=dt0';
    dt_=[dt0;temp];%gt
    t(i,:)=[dt_(1,4),dt_(3,4)];
   if i>=2
    dt1=reshape(dt(i-1,:),4,3);
    dt1=dt1';
    dt_1=[dt1;temp];%地面真实值
      dist_temp=dt0(:,4)-dt1(:,4);%两帧相对平移
    dist=sqrt(dist_temp'*dist_temp)+dist;%总运动路程
   else
        dt1=reshape(dt(1,:),4,3);
    dt1=dt1';
    dt_1=[dt1;temp];%地面真实值
      dist_temp=dt0(:,4)-dt1(:,4);%两帧相对平移
    dist=sqrt(dist_temp'*dist_temp)+dist;%总运动路程
   end

%%error test
    %distance
  
    
    length(i)=dist;
     pose_error=inv(d)*(dt_);
      %translationError
      dx=pose_error(1,4);
      dy=pose_error(2,4);
      dz=pose_error(3,4);
      translationError(i)=(sqrt(dx*dx+dy*dy+dz*dz)/dist)*100;
      %rotationError
      a = pose_error(1,1);
      b = pose_error(2,2);
      c = pose_error(3,3);
      d = 0.5*(a+b+c-1.0);
      rotationError(i)=acos(max(min(d,1.0),-1.0))/dist;
      
end

save 00data rotationError translationError t1 t length;%保存数据
figure
   plot(t1(:,1),t1(:,2),t(:,1),t(:,2), 'LineWidth',2);
       title('实验测试结果');
       ylabel('Z(m)');xlabel('X(m)');
      legend('实验结果轨迹','真实轨迹');
      
  figure
  subplot(2,1,1);
     plot(length,translationError);
       title('平移误差'); legend('实验相对平移误差分析');
       ylabel('相对平移误差(%)');xlabel('运动里程(m)');
    
    subplot(2,1,2);

     plot(length,rotationError);
       title('旋转误差');legend('实验相对旋转误差分析');
       ylabel('相对旋转误差（rad/m）');xlabel('运动里程(m)');


      
      