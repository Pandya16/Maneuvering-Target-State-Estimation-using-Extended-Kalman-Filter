% Extended Kalman Filter with (1/beta(dot) = 0) for Maneuvering Target

clc; close all; clear all;

Measurements = load("Measurements2.txt");
range = Measurements(:, 2);
theta = Measurements(:, 3);
phi = Measurements(:, 4);
Rdot = Measurements(:, 5);

measurements = [range theta phi Rdot];
dt = 0.01;
x_cap = [100000.0; 10000.0; 1000000.0; 100.0; -25.0; -2000.0; 1/14000; 0.0; -0.0];
P_cap = diag([100000 100000 100000 1000 1000 1000 1/3000 1e-8 1e-8]);
g = 9.81;
dens = -0.0001516584;

num_measurements = length(range);
estimated_states = zeros(9, num_measurements);
estimated_errors = zeros(9, num_measurements);
estimated_pcapp = zeros(9, num_measurements);
estimated_pcapn = zeros(9, num_measurements);
innovations = zeros(4, num_measurements);

Data_Target = load("Target_Trajectory_Man.txt");
x_traj = Data_Target(:, 2);
y_traj = Data_Target(:, 3);
z_traj = Data_Target(:, 4);
x_vel = Data_Target(:, 5);
y_vel = Data_Target(:, 6);
z_vel = Data_Target(:, 7);
beta = Data_Target(:, 8);
z1 = Data_Target(:, 9);
z2 = Data_Target(:, 10);
time = Data_Target(:, 1);

true_states = [x_traj y_traj z_traj x_vel y_vel z_vel beta z1 z2]';

h = @(x) [sqrt(x(1)^2+x(2)^2+x(3)^2), atan2(x(2),x(3)), atan2(x(1),sqrt(x(2)^2+x(3)^2)), (x(1)*x(4)+x(2)*x(5)+x(3)*x(6))/sqrt(x(1)^2+x(2)^2+x(3)^2)];

H = @(x) [x(1)/sqrt(x(1)^2+x(2)^2+x(3)^2), x(2)/sqrt(x(1)^2 + x(2)^2 + x(3)^2), x(3)/sqrt(x(1)^2 + x(2)^2 + x(3)^2),0,0,0,0,0,0;
        0,1/sqrt(x(2)^2+x(3)^2),-1*x(2)/(x(3)*sqrt(x(2)^2+x(3)^2)),0,0,0,0,0,0;
        1/sqrt(x(1)^2+x(2)^2+x(3)^2),-1*x(1)*x(2)/(sqrt(x(1)^2+x(2)^2+x(3)^2)*sqrt(x(2)^2+x(3)^2)),-1*x(1)*x(3)/(sqrt(x(1)^2+x(2)^2+x(3)^2)*sqrt(x(2)^2+x(3)^2)),0,0,0,0,0,0;
        (x(4)*sqrt(x(1)^2+x(2)^2+x(3)^2)-(x(1)*(x(1)*x(4)+x(2)*x(5)+x(3)*x(6))/sqrt(x(1)^2+x(2)^2+x(3)^2)))/(x(1)^2+x(2)^2+x(3)^2),(x(5)*sqrt(x(1)^2+x(2)^2+x(3)^2)-(x(2)*(x(1)*x(4)+x(2)*x(5)+x(3)*x(6))/sqrt(x(1)^2+x(2)^2+x(3)^2)))/(x(1)^2+x(2)^2+x(3)^2),(x(6)*sqrt(x(1)^2+x(2)^2+x(3)^2)-(x(3)*(x(1)*x(4)+x(2)*x(5)+x(3)*x(6))/sqrt(x(1)^2+x(2)^2+x(3)^2)))/(x(1)^2+x(2)^2+x(3)^2),x(1)/sqrt(x(1)^2+x(2)^2+x(3)^2),x(2)/sqrt(x(1)^2+x(2)^2+x(3)^2),x(3)/sqrt(x(1)^2+x(2)^2+x(3)^2),0,0,0;
    ];

A = @(x) [0,0,0,1,0,0,0,0,0;
          0,0,0,0,1,0,0,0,0;
          0,0,0,0,0,1,0,0,0;
          0.5*dens*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*(-x(4)*x(7)-x(9)*sqrt(x(5)^2+x(6)^2)),0,0,0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(7)-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)^2*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)*x(9)*sqrt(x(5)^2+x(6)^2)/(sqrt(x(4)^2+x(5)^2+x(6)^2)),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)*x(5)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(9)*x(5)*sqrt(x(5)^2+x(6)^2)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(9)*x(5)/sqrt(x(5)^2+x(6)^2),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)*x(6)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(9)*x(6)*sqrt(x(5)^2+x(6)^2)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(9)*x(6)/sqrt(x(5)^2+x(6)^2),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(4),0,-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*sqrt(x(5)^2+x(6)^2);
          0.5*dens*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))^2*((-x(5)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+(x(6)*x(8)/sqrt(x(5)^2+x(6)^2))+x(4)*x(5)*x(9)/((sqrt(x(4)^2+x(5)^2+x(6)^2))*sqrt(x(5)^2+x(6)^2))),0,0,0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*((-x(4)*x(5)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+2*x(4)*x(6)*x(8)/sqrt(x(5)^2+x(6)^2)+x(5)*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(9)/sqrt(x(5)^2+x(6)^2)+x(5)*x(9)*x(4)^2/((sqrt(x(4)^2+x(5)^2+x(6)^2))*sqrt(x(5)^2+x(6)^2))),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(7)-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(5)^2*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2))+0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(6)*x(8)*(2*x(5)*(x(5)^2+x(6)^2)-(sqrt(x(4)^2+x(5)^2+x(6)^2))^2*x(5))/(x(5)^2+x(6)^2)^(3/2)+0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(9)*x(4)*((sqrt(x(4)^2+x(5)^2+x(6)^2))*x(6)^2+x(5)^4/(sqrt(x(4)^2+x(5)^2+x(6)^2))+(x(5)*x(6))^2/(sqrt(x(4)^2+x(5)^2+x(6)^2)))/(x(5)^2+x(6)^2)^(3/2),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*((-x(5)*x(6)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+x(8)*(((sqrt(x(4)^2+x(5)^2+x(6)^2))*x(5))^2+2*x(6)^2*x(5)^2+2*x(6)^4)/(x(5)^2+x(6)^2)^(3/2)+x(4)*x(5)*x(9)*((x(5)^2+x(6)^2)*x(6)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(6))/(x(5)^2+x(6)^2)^(3/2)),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(5)*(sqrt(x(4)^2+x(5)^2+x(6)^2)),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(6)*((sqrt(x(4)^2+x(5)^2+x(6)^2))^2)/sqrt(x(5)^2+x(6)^2),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)*x(5)*sqrt(x(4)^2+x(5)^2+x(6)^2)/sqrt(x(5)^2+x(6)^2);
          0.5*dens*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))^2*((-x(6)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))-(x(5)*x(8)/sqrt(x(5)^2+x(6)^2))+x(4)*x(6)*x(9)/((sqrt(x(4)^2+x(5)^2+x(6)^2))*sqrt(x(5)^2+x(6)^2))),0,0,0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*((-x(4)*x(6)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))-2*x(4)*x(5)*x(8)/sqrt(x(5)^2+x(6)^2)+x(6)*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(9)/sqrt(x(5)^2+x(6)^2)+x(6)*x(9)*x(4)^2/((sqrt(x(4)^2+x(5)^2+x(6)^2))*sqrt(x(5)^2+x(6)^2))),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*((-x(5)*x(6)*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2)))-x(8)*(((sqrt(x(4)^2+x(5)^2+x(6)^2))*x(6))^2+2*x(6)^2*x(5)^2+2*x(5)^4)/(x(5)^2+x(6)^2)^(3/2)+x(4)*x(6)*x(9)*((x(5)^2+x(6)^2)*x(5)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(5))/(x(5)^2+x(6)^2)^(3/2)),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(7)-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(6)^2*x(7)/(sqrt(x(4)^2+x(5)^2+x(6)^2))-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(5)*x(8)*(2*x(6)*(x(5)^2+x(6)^2)+(sqrt(x(4)^2+x(5)^2+x(6)^2))^2*x(6))/(x(5)^2+x(6)^2)^(3/2)+0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(9)*x(4)*((sqrt(x(4)^2+x(5)^2+x(6)^2))*x(5)^2+x(6)^4/(sqrt(x(4)^2+x(5)^2+x(6)^2))+(x(5)*x(6))^2/(sqrt(x(4)^2+x(5)^2+x(6)^2)))/(x(5)^2+x(6)^2)^(3/2),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(6)*(sqrt(x(4)^2+x(5)^2+x(6)^2)),-0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(5)*((sqrt(x(4)^2+x(5)^2+x(6)^2))^2)/sqrt(x(5)^2+x(6)^2),0.5*(0.1819*2*exp(-0.0001516584*((x(1))-11000)))*x(4)*x(6)*sqrt(x(4)^2+x(5)^2+x(6)^2)/sqrt(x(5)^2+x(6)^2);
          0,0,0,0,0,0,0,0,0;
          0,0,0,0,0,0,0,0,0;
          0,0,0,0,0,0,0,0,0;
    ];

Qd = 1*diag([0.1 0.1 0.1 0.2 0.2 0.2 1e-18 1e-15 1e-15]);
I = eye(9);

for i = 1:num_measurements
    if(i==1)
        fact_R = 10000000;
    else
        fact_R = min(max(1000/time(i),100),1000);
    end

    R = diag([2500 1e-5 1e-5 50]*fact_R);

    k1 = @(x, delt, k, n) (delt*(x+(n*k)));
    k1v = @(delt, n, k1, k2, k3, k4) delt*(0.5*(0.1819*2*exp(-0.0001516584*((x_cap(1)+n*k4)-11000)))*(sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2))*((-1*(x_cap(4)+n*k1)*x_cap(7))-(x_cap(9)*sqrt((x_cap(5)+n*k2)^2+(x_cap(6)+n*k3)^2)))-g);
    k2v = @(delt, n, k1, k2, k3, k4) delt*0.5*(0.1819*2*exp(-0.0001516584*((x_cap(1)+n*k4)-11000)))*((sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2))^2)*((-1*(x_cap(5)+n*k2)*x_cap(7)/(sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2)))+((x_cap(6)+n*k3)*x_cap(8)/sqrt((x_cap(5)+n*k2)^2+(x_cap(6)+n*k3)^2))+((x_cap(4)+n*k1)*(x_cap(5)+n*k2)*x_cap(9)/((sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2))*sqrt((x_cap(5)+n*k2)^2+(x_cap(6)+n*k3)^2))));
    k3v = @(delt, n, k1, k2, k3, k4) delt*0.5*(0.1819*2*exp(-0.0001516584*((x_cap(1)+n*k4)-11000)))*((sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2))^2)*((-1*(x_cap(6)+n*k3)*x_cap(7)/(sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2)))-((x_cap(5)+n*k2)*x_cap(8)/sqrt((x_cap(5)+n*k2)^2+(x_cap(6)+n*k3)^2))+((x_cap(4)+n*k1)*(x_cap(6)+n*k3)*x_cap(9)/((sqrt((x_cap(4)+n*k1)^2 + (x_cap(5)+n*k2)^2 + (x_cap(6)+n*k3)^2))*sqrt((x_cap(5)+n*k2)^2+(x_cap(6)+n*k3)^2))));

    k1x = k1(x_cap(4),dt,0,0);
    k1y = k1(x_cap(5),dt,0,0);
    k1z = k1(x_cap(6),dt,0,0);
    k1vx = k1v(dt,0,0,0,0,0);
    k1vy = k2v(dt,0,0,0,0,0);
    k1vz = k3v(dt,0,0,0,0,0);

    k2x = k1(x_cap(4),dt,k1vx,0.5);
    k2y = k1(x_cap(5),dt,k1vy,0.5);
    k2z = k1(x_cap(6),dt,k1vz,0.5);
    k2vx = k1v(dt,0.5,k1vx,k1vy,k1vz,k1x);
    k2vy = k2v(dt,0.5,k1vx,k1vy,k1vz,k1x);
    k2vz = k3v(dt,0.5,k1vx,k1vy,k1vz,k1x);

    k3x = k1(x_cap(4),dt,k2vx,0.5);
    k3y = k1(x_cap(5),dt,k2vy,0.5);
    k3z = k1(x_cap(6),dt,k2vz,0.5);
    k3vx = k1v(dt,0.5,k2vx,k2vy,k2vz,k2x);
    k3vy = k2v(dt,0.5,k2vx,k2vy,k2vz,k2x);
    k3vz = k3v(dt,0.5,k2vx,k2vy,k2vz,k2x);

    k4x = k1(x_cap(4),dt,k3vx,1);
    k4y = k1(x_cap(5),dt,k3vy,1);
    k4z = k1(x_cap(6),dt,k3vz,1);
    k4vx = k1v(dt,1,k3vx,k3vy,k3vz,k3x);
    k4vy = k2v(dt,1,k3vx,k3vy,k3vz,k3x);
    k4vz = k3v(dt,1,k3vx,k3vy,k3vz,k3x);

    f = @(x) [x(1) + (1/6)*(k1x+2*k2x+2*k3x+k4x);
              x(2) + (1/6)*(k1y+2*k2y+2*k3y+k4y);
              x(3) + (1/6)*(k1z+2*k2z+2*k3z+k4z);
              x(4) + (1/6)*(k1vx+2*k2vx+2*k3vx+k4vx);
              x(5) + (1/6)*(k1vy+2*k2vy+2*k3vy+k4vy);
              x(6) + (1/6)*(k1vz+2*k2vz+2*k3vz+k4vz);
              x(7); x(8); x(9);
        ];

    if(i==1)
        x_bar = x_cap;
    else
        x_bar = f(x_cap);
    end

    B = A(x_cap);
    D = eye(9) + B*dt;
    P_bar = D*P_cap*D' + Qd;

    y_meas = measurements(i,:);
   
    y_bar = h(x_bar);
    J = H(x_cap);
    S = J*P_bar*J' + R;
    K = P_bar*J'/S;
    P_cap = (I-K*J)*P_bar;
    Y = y_meas - y_bar;
    x_cap = x_bar + 1.0*K*Y';
    estimated_states(:, i) = x_cap;
    estimated_errors(:, i) = (true_states(:,i) - x_cap);
    estimated_pcapp(:, i) = sqrt(diag(P_cap));
    estimated_pcapn(:, i) = -1*sqrt(diag(P_cap));
    innovations(:,i) = Y';
end

disp(1/x_cap(7)); disp(x_cap(8)); disp(x_cap(9));

% ----------------------------------------------------------------
% Plot all neccessary graphs - code available in Plot_temp.m files
% ----------------------------------------------------------------

figure;
plot(time, estimated_states(1,:), 'DisplayName', 'Estimated States', 'LineWidth', 2);
hold on;
plot(time, x_traj, 'DisplayName', 'Trajectory');
hold off;
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; % Set the grid color to black
ax.GridAlpha = 1; % Adjust the transparency of the grid lines
legend show;