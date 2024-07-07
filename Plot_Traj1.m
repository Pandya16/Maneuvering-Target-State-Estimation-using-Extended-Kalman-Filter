% Template file to plot the graphs of true states vs estimated states
% Use directly after EKF1 and EKF2 Files

clc; close all; clear all;

% For 3D Plot of Trajectory
figure;
plot3(z_traj,y_traj,x_traj,'b');
hold on;
plot3(estimated_states(3,:),estimated_states(2,:),estimated_states(1,:),'r');
hold off;
xlabel('Z'); ylabel('Y'); zlabel('X');
title('3D Plot of Trajectory');
legend('Actual','Estimated');
grid on;

% X Position Plot
figure;
plot(time, x_traj, time, estimated_states(1,:));
xlabel(time);
ylabel('X Coordinate');
title('Estimation of X Position');
legend('Actual','Estimated');
grid on;

% Y Position Plot
figure;
plot(time, y_traj, time, estimated_states(2,:));
xlabel('Time');
ylabel('Y Coordinate');
title('Estimation of Y Position');
legend('Actual','Estimated');
grid on;

% Z Position Plot
figure;
plot(time, z_traj, time, estimated_states(3,:));
xlabel('Time');
ylabel('Z Coordinate');
title('Estimation of Z Position');
legend('Actual','Estimated');
grid on;

% X Velocity Plot
figure;
plot(time, x_vel, time, estimated_states(4,:));
xlabel('Time');
ylabel('X Velocity');
title('Estimation of X Velocity');
legend('Actual','Estimated');
grid on;

% Y Velocity Plot
figure;
plot(time, y_vel, time, estimated_states(5,:));
xlabel('Time');
ylabel('Y Velocity');
title('Estimation of Y Velocity');
legend('Actual','Estimated');
grid on;

% Z Velocity Plot
figure;
plot(time, z_vel, time, estimated_states(6,:));
xlabel('Time');
ylabel('Z Velocity');
title('Estimation of Z Velocity');
legend('Actual','Estimated');
grid on;

% Drag Coefficient Plot
figure;
plot(time, beta, time, estimated_states(7,:));
xlabel('Time');
ylabel('Drag Coefficient');
title('Estimation of Drag Coeff.');
legend('Actual','Estimated');
grid on;

% Plot of Aerodynamic Lift Coefficient Z_1
figure;
plot(time, z1, time, estimated_states(8,:));
xlabel('Time');
ylabel('Z_1');
title('Estimation of Z_1');
legend('Actual','Estimated');
grid on;

% Plot of Aerodynamic Lift Coefficient Z_2
figure;
plot(time, z2, time, estimated_states(9,:));
xlabel('Time');
ylabel('Z_2');
title('Estimation of Z_2');
legend('Actual','Estimated');
grid on;

% The grid could be changed accordingly with the code given below
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; % Set the grid color to black
ax.GridAlpha = 1; % Adjust the transparency of the grid lines
legend show;