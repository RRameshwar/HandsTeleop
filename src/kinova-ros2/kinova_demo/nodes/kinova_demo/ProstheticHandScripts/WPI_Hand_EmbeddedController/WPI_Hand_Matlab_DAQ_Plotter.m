
% load('04-25-2018 22-12_Grasp_Manual.mat')
% load('04-25-2018 22-14_Grasp_Manual.mat')


% load('04-27-2018 20-02_BottlePickPlace_150gr_Dragonskin30_GreenDownForce.mat')
% load('04-27-2018 19-57_BottlePickPlace_150gr_Ecolex0050_GreenDownForce.mat')
% load('04-27-2018 20-08_BottlePickPlace_250gr_Dragonskin30.mat')
% load('04-27-2018 19-53_BottlePickPlace_250gr_Ecolex0050.mat')
load('04-27-2018 20-17_EggPickPlace.mat')

% figure
% subplot(2,1,1)
% grid on
% hold on
% plot(T_Array/1000, fingerAngleArray, 'b','linewidth', 2)
% plot(T_Array/1000, fingerAngleRefArray, 'r','linewidth', 2)
% axis([0 10 0 90])
% xlabel('Time [s]')
% ylabel('Angle [deg]')
% set(gca,'FontSize', 14);
% legend('Angle - Actual', 'Angle - Reference')
% 
% subplot(2,1,2)
% plot(T_Array/1000,motorPWMArray, 'k','linewidth', 2)
% grid on
% axis([0 10 -3*1023 3*1023])
% xlabel('Time [s]')
% ylabel('PWM Value [a.u]')
% set(gca,'FontSize', 14);

figure
subplot(2,1,1)
grid on
hold on
plot(T_Array/1000, thumbB_AngleArray, 'b', 'linewidth',2)
plot(T_Array/1000, thumbB_AngleRefArray, 'r', 'linewidth',2)
axis([0 end_time 0 90])
xlabel('Time [s]')
ylabel('Angle [deg]')
set(gca,'FontSize', 14);
legend('Angle - Actual', 'Angle - Reference')
title('Thumb')

subplot(2,1,2)
grid on
hold on
plot(T_Array./1000, index_AngleArray, 'b', 'linewidth',2)
plot(T_Array./1000, index_AngleRefArray, 'r', 'linewidth',2)
axis([0 end_time 0 90])
xlabel('Time [s]')
ylabel('Angle [deg]')
set(gca,'FontSize', 14);
legend('Angle - Actual', 'Angle - Reference')
title('Index')

% Fix Bx, By, Bz data
% Bx_Array_Fixed = zeros(1,length(Bx_Array));
% By_Array_Fixed = zeros(1,length(By_Array));
% Bz_Array_Fixed = zeros(1,length(Bz_Array));
% Bx_Array_Fixed(1) = Bx_Array(1);
% By_Array_Fixed(1) = By_Array(1);
% Bz_Array_Fixed(1) = Bz_Array(1);
% for i = 2:length(Bx_Array)
%     if(abs(Bx_Array(i) - Bx_Array(i-1))>1000)
%         Bx_Array_Fixed(i) =  Bx_Array(i) - Bx_Array(i-1)
%     else
%         Bx_Array_Fixed(i) = Bx_Array(i);
%     end
%     
% end

figure
grid on
hold on
xlabel('Time [s]')
ylabel('ADC Magnetic Field')
set(gca,'FontSize', 14);
plot(T_Array/1000, Bx_Array+40, 'r', 'linewidth',2)
plot(T_Array/1000, By_Array-172+26, 'g', 'linewidth',2)
plot(T_Array/1000, Bz_Array+515, 'b', 'linewidth',2)
plot(T_Array/1000, thumbB_AngleRefArray-15, '-k', 'linewidth',3)
legend('Shear - Front', 'Shear - Down', 'Axial', 'Open/Close Trigger')
% axis([0 end_time -2.5 2.5])
