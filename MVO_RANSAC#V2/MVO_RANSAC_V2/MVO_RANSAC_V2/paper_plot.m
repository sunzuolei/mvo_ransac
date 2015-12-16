  clc;clear all;close all;
data0=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\00data');
data3=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\03data');
data9=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\09data');

d0=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\data00');

d3=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\data03');
d9=load('F:\Huang Jiaming\mvo_ransac\mvo_ransac\mvo_ransac\data09');

for i=1:n

end
figure
subplot(1,3,1)
   plot(data0.t1(:,1),data0.t1(:,2),'-.',data0.t(:,1),data0.t(:,2),'--',d0.t1(:,1),d0.t1(:,2), 'LineWidth',2);
       title('00组实验测试结果');
       ylabel('Z(m)');xlabel('X(m)');
       legend('RANSAC实验结果','真实轨迹','未使用RANSAC');
      subplot(1,3,2)
       plot(data3.t1(:,1),data3.t1(:,2),'-.',data3.t(:,1),data3.t(:,2),'--',d3.t1(:,1),d3.t1(:,2), 'LineWidth',2);
       title('03组实验测试结果');
       ylabel('Z(m)');xlabel('X(m)');
     legend('RANSAC实验结果','真实轨迹','未使用RANSAC');
      subplot(1,3,3)
       plot(data9.t1(:,1),data9.t1(:,2),'-.',data9.t(:,1),data9.t(:,2),'--',d9.t1(:,1),d9.t1(:,2),  'LineWidth',2);
       title('09组实验测试结果');
       ylabel('Z(m)');xlabel('X(m)');
      legend('RANSAC实验结果','真实轨迹','未使用RANSAC');
figure
 subplot(2,3,1);
     plot(data0.length,data0.translationError);
       title('00组平移误差分析'); 
       ylabel('相对平移误差(%)');xlabel('运动里程(m)');
        
       
       subplot(2,3,2);
     plot(data3.length,data3.translationError);
       title('03组平移误差分析'); 
       ylabel('相对平移误差(%)');xlabel('运动里程(m)');
    

        subplot(2,3,3);
     plot(data9.length,data9.translationError);
       title('09组平移误差分析'); 
       ylabel('相对平移误差(%)');xlabel('运动里程(m)');
    
       
    subplot(2,3,4);
     plot(data0.length,data0.rotationError);
       title('00组旋转误差');
       ylabel('相对旋转误差（rad）');xlabel('运动里程(m)');
       
    
    subplot(2,3,5);
     plot(data3.length,data3.rotationError);
       title('03组旋转误差分析');
       ylabel('相对旋转误差（rad）');xlabel('运动里程(m)');

    subplot(2,3,6);
     plot(data9.length,data9.rotationError);
       title('09组旋转误差分析');
       ylabel('相对旋转误差（rad）');xlabel('运动里程(m)');
