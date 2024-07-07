% Template file to plot the graphs of Covariances vs estimated errors
% Use directly after EKF1 and EKF2 Files
% The error should fall within the 1-sigma bound for 66.6% of the observations.
% The error should fall within the 3-sigma bound for 99.9% of the observations.

clc; close all; clear all;

figure;
plot(time, estimated_errors(1,:),'b');
hold on;
plot(time, estimated_pcapp(1,:), 'r', time, estimated_pcapn(1,:), 'r');
hold off;
xlabel('Time');
ylabel('X Coordinate');
title('X Position Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(2,:),'b');
hold on;
plot(time, estimated_pcapp(2,:), 'r', time, estimated_pcapn(2,:), 'r');
hold off;
xlabel('Time');
ylabel('Y Coordinate');
title('Y Position Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(3,:),'b');
hold on;
plot(time, estimated_pcapp(3,:), 'r', time, estimated_pcapn(3,:), 'r');
hold off;
xlabel('Time');
ylabel('Z Coordinate');
title('Z Position Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(4,:),'b');
hold on;
plot(time, estimated_pcapp(4,:), 'r', time, estimated_pcapn(4,:), 'r');
hold off;
xlabel('Time');
ylabel('X Velocity');
title('X Velocity Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(5,:),'b');
hold on;
plot(time, estimated_pcapp(5,:), 'r', time, estimated_pcapn(5,:), 'r');
hold off;
xlabel('Time');
ylabel('Y Velocity');
title('Y Velocity Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(6,:),'b');
hold on;
plot(time, estimated_pcapp(6,:), 'r', time, estimated_pcapn(6,:), 'r');
hold off;
xlabel('Time');
ylabel('Z Velocity');
title('Z Velocity Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(7,:),'b');
hold on;
plot(time, estimated_pcapp(7,:), 'r', time, estimated_pcapn(7,:), 'r');
hold off;
xlabel('Time');
ylabel('Drag Coefficient');
title('Drag Coefficient Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(8,:),'b');
hold on;
plot(time, estimated_pcapp(8,:), 'r', time, estimated_pcapn(8,:), 'r');
hold off;
xlabel('Time');
ylabel('Z_1');
title('Z_1 Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

figure;
plot(time, estimated_errors(9,:),'b');
hold on;
plot(time, estimated_pcapp(9,:), 'r', time, estimated_pcapn(9,:), 'r');
hold off;
xlabel('Time');
ylabel('Z_2');
title('Z_2 Estimation Error');
legend('Estimation Error', 'Covariance');
grid on;

% The grid could be changed accordingly with the code given below
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; % Set the grid color to black
ax.GridAlpha = 1; % Adjust the transparency of the grid lines
legend show;