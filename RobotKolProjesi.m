clc;
clear;
close all;
clear classes;

%% =========================
% 1. TOOLBOX BAŞLAT
% =========================
startup_rvc;

%% =========================
% 2. ROBOT TANIMI
% =========================
L1 = 10;
L2 = 10;
L3 = 10;

L(1) = Link('d', L1, 'a', 0,  'alpha', pi/2);
L(2) = Link('d', 0,  'a', L2, 'alpha', 0);
L(3) = Link('d', 0,  'a', L3, 'alpha', 0);

robot = SerialLink(L, 'name', '3DOF Robot');

figure;
robot.plot([0 0 0]);
title('Başlangıç Pozisyonu');

%% =========================
% 3. İLERİ KİNEMATİK
% =========================
q_test = [pi/4 pi/4 pi/4];
T = robot.fkine(q_test);

disp('--- İleri Kinematik ---');
disp(T);

%% =========================
% 4. TERS KİNEMATİK (MANUEL)
% =========================
x = 10;
y = 10;
z = 15;

theta1 = atan2(y, x);

r = sqrt(x^2 + y^2);
z_ = z - L1;

d = sqrt(r^2 + z_^2);

D = (d^2 - L2^2 - L3^2)/(2*L2*L3);
theta3 = atan2(sqrt(1-D^2), D);

theta2 = atan2(z_, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3));

q_end = [theta1 theta2 theta3];

disp('--- IK Açılar ---');
disp(q_end);

figure;
robot.plot(q_end);
title('Ters Kinematik Sonucu');

%% =========================
% 5. JOINT TRAJECTORY
% =========================
q_start = [0 0 0];
t = 50;

q_traj = jtraj(q_start, q_end, t);

figure;
robot.plot(q_traj(1,:));  % sadece bir kez çiz

for i = 1:t
    robot.animate(q_traj(i,:)); % doğru kullanım
    drawnow;
end
title('Joint Trajectory Hareketi');

%% =========================
% 6. TRAJECTORY GRAFİĞİ
% =========================
figure;
plot(q_traj, 'LineWidth', 2);
grid on;
title('Joint Trajectory Grafiği');
xlabel('Adım');
ylabel('Açı (rad)');
legend('θ1','θ2','θ3');

%% =========================
% 7. CARTESIAN TRAJECTORY (FIXED)
% =========================
T1 = robot.fkine(q_start);
T2 = robot.fkine(q_end);

T_traj = ctraj(T1, T2, t);

figure;
robot.plot(q_start); % sadece bir kez çiz

for i = 1:t
    q = robot.ikine(T_traj(i), 'mask', [1 1 1 0 0 0]); % DÜZELTİLDİ
    robot.animate(q);
    drawnow;
end
title('Cartesian Trajectory');

%% =========================
% 8. STM32 İÇİN VERİ
% =========================
angles_deg = q_traj * 180/pi;

disp('--- STM32 açı verisi ---');
disp(angles_deg);

writematrix(angles_deg, 'trajectory_data.csv');

disp('Veriler trajectory_data.csv dosyasına kaydedildi');
