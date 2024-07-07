% Measurement Model of Radar to record the Measurements for Maneuvering Target

clc; close all; clear all;

g = 9.81;

fid = fopen("Measurements2.txt", 'w');
Data_Target = load("Target_Trajectory_Man.txt");

x_traj = Data_Target(:,2);
y_traj = Data_Target(:,3);
z_traj = Data_Target(:,4);
x_vel = Data_Target(:,5);
y_vel = Data_Target(:,6);
z_vel = Data_Target(:,7);
time = Data_Target(:,1);

% Measurement Noise
sigma_r = 50;
sigma_theta = 1e-4;
sigma_phi = 1e-4;
sigma_Rdot = 5;

num_points = length(x_traj);
R_measured = zeros(1, num_points);
theta_measured = zeros(1, num_points);
phi_measured = zeros(1, num_points);
R_dot = zeros(1, num_points);

for i = 1:num_points
    R = sqrt(x_traj(i)^2 + y_traj(i)^2 + z_traj(i)^2);
    theta = atan2(y_traj(i), z_traj(i));
    phi = atan2(x_traj(i), sqrt(y_traj(i)^2 + z_traj(i)^2));
    
    R_measured(i) = R + sigma_r * randn;
    theta_measured(i) = (theta) + sigma_theta * randn;
    phi_measured(i) = (phi) + sigma_phi * randn;
    R_dot(i) = (x_traj(i)*x_vel(i) + y_traj(i)*y_vel(i) + z_traj(i)*z_vel(i))/R + sigma_Rdot * randn;
    fprintf(fid,'%6.6f %6.6f %6.6f %6.6f %6.6f\n', time(i), R_measured(i), theta_measured(i), phi_measured(i), R_dot(i));
end

fclose(fid);

figure;
subplot(4,1,1);
plot(time, R_measured);
xlabel('Time');
ylabel('Range (m)');
title('Range Measurement');
grid on;

subplot(4,1,2);
plot(time, theta_measured);
xlabel('Time');
ylabel('Azimuth Angle');
title('Azimuth Angle Measurement');
grid on;

subplot(4,1,3);
plot(time, phi_measured);
xlabel('Time');
ylabel('Elevation Angle');
title('Elevation Angle Measurement');
grid on;

subplot(4,1,4);
plot(time, R_dot);
xlabel('Time');
ylabel('Velocity');
title('Range Rate Measurement');
grid on;
