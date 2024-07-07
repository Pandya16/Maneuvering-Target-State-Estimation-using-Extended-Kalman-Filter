% Template file to plot the graphs of Innovation and Gains
% Use directly after EKF1 and EKF2 Files
% The innovation plots should consistently exhibit a zero mean.
% The gain matrix is a 7x4 matrix; plots should be generated for specific rows accordingly.

% Plot for Innovations
figure;
plot(time, innovations(1,:));
xlabel('Time');
ylabel('Range');
title('Innovation - Range')
grid on;

figure;
plot(time, innovations(2,:));
xlabel('Time');
ylabel('\theta');
title('Innovation - Theta')
grid on;

figure;
plot(time, innovations(3,:));
xlabel('Time');
ylabel('\phi');
title('Innovation - Phi')
grid on;

figure;
plot(time, innovations(4,:));
xlabel('Time');
ylabel('Range Rate');
title('Innovation - Range Rate')
grid on;