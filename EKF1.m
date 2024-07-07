% Extended Kalman Filter with (1/beta(dot) = 0) for Ballistic Target

clc; clear all; close all;

Measurements = load("Measurements.txt");
range = Measurements(:, 2);
theta = Measurements(:, 3);
phi = Measurements(:, 4);

measurements = [range theta phi];
dt = 0.01;
x_cap = [85570.0; 67885.0; 74770.0; 350.0; -95.0; -355.0; 1/12000];
P_cap = diag([200000 200000 200000 10000 10000 10000 1e-6]);
R = diag([10000 5*1e-3 5*1e-3]);
g = 9.81;
dens = 0.0001516584;

num_measurements = length(range);
estimated_states = zeros(7, num_measurements);
estimated_errors = zeros(7, num_measurements);
estimated_pcapp = zeros(7, num_measurements);
estimated_pcapn = zeros(7, num_measurements);

Data_Target = load("Target_Trajectory.txt");
x_traj = Data_Target(:, 2);
y_traj = Data_Target(:, 3);
z_traj = Data_Target(:, 4);
x_vel = Data_Target(:, 5);
y_vel = Data_Target(:, 6);
z_vel = Data_Target(:, 7);
beta = Data_Target(:, 8);
time = Data_Target(:, 1);

true_states = [x_traj y_traj z_traj x_vel y_vel z_vel beta]';

A = @(x) [0,0,0,1,0,0,0;
          0,0,0,0,1,0,0;
          0,0,0,0,0,1,0;
          dens*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(4)*x(7)/(2),0,0,-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*((x(4)^2/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+(sqrt(x(4)^2+x(5)^2+x(6)^2)))*x(7)/(2),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(4)*x(5)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(4)*x(6)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-1*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(4)*(sqrt(x(4)^2+x(5)^2+x(6)^2))/(2);
          dens*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(5)*x(7)/(2),0,0,-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(4)*x(5)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*((x(5)^2/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+(sqrt(x(4)^2+x(5)^2+x(6)^2)))*x(7)/(2),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(5)*x(6)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-1*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(5)*(sqrt(x(4)^2+x(5)^2+x(6)^2))/(2);
          dens*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*(sqrt(x(4)^2+x(5)^2+x(6)^2))*x(6)*x(7)/(2),0,0,-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(4)*x(6)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(5)*x(6)*x(7)/(2*(sqrt(x(4)^2+x(5)^2+x(6)^2))),-(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*((x(6)^2/(sqrt(x(4)^2+x(5)^2+x(6)^2)))+(sqrt(x(4)^2+x(5)^2+x(6)^2)))*x(7)/(2),-1*(0.1819*2*exp(-0.0001516584*(x(1)-11000)))*x(6)*(sqrt(x(4)^2+x(5)^2+x(6)^2))/(2);
          0,0,0,0,0,0,0;
         ];

h = @(x) [sqrt(x(1)^2+x(2)^2+x(3)^2), atan2(x(2),x(3)), atan2(x(1),sqrt(x(2)^2+x(3)^2))];

H = @(x) [x(1)/sqrt(x(1)^2+x(2)^2+x(3)^2), x(2)/sqrt(x(1)^2 + x(2)^2 + x(3)^2), x(3)/sqrt(x(1)^2 + x(2)^2 + x(3)^2), 0,0,0,0;
    0,1/sqrt(x(2)^2+x(3)^2),-1*x(2)/(x(3)*sqrt(x(2)^2+x(3)^2)),0,0,0,0;
    1/sqrt(x(1)^2+x(2)^2+x(3)^2),-1*x(1)*x(2)/(sqrt(x(1)^2+x(2)^2+x(3)^2)*sqrt(x(2)^2+x(3)^2)),-1*x(1)*x(3)/(sqrt(x(1)^2+x(2)^2+x(3)^2)*sqrt(x(2)^2+x(3)^2)),0,0,0,0;
    ];

Qd = 0.1*diag([0 0 0 0.1 0.1 0.1 1e-15]);
I = eye(7);

for k = 1:num_measurements
    rho = 0.1819*2*exp(-0.0001516584*(x_cap(1)-11000));
    vt = sqrt(x_cap(4)^2+x_cap(5)^2+x_cap(6)^2);

    k1 = @(x, delt, k, n) (delt*(x+(n*k)));
    k1v = @(x, delt, k, n, m) (delt*((-1*rho*vt*(x+(n*k))*x_cap(7)/(2))-m*g));

    k1x = k1(x_cap(4), dt, 0, 0);
    k1y = k1(x_cap(5), dt, 0, 0);
    k1z = k1(x_cap(6), dt, 0, 0);
    k1vx = k1v(x_cap(4), dt, 0, 0, 1);
    k1vy = k1v(x_cap(5), dt, 0, 0, 0);
    k1vz = k1v(x_cap(6), dt, 0, 0, 0);
    k1b = 0;

    k2x = k1(x_cap(4), dt, k1vx, 0.5);
    k2y = k1(x_cap(5), dt, k1vy, 0.5);
    k2z = k1(x_cap(6), dt, k1vz, 0.5);
    k2vx = k1v(x_cap(4), dt, k1vx, 0.5, 1);
    k2vy = k1v(x_cap(5), dt, k1vy, 0.5, 0);
    k2vz = k1v(x_cap(6), dt, k1vz, 0.5, 0);
    k2b = 0;

    k3x = k1(x_cap(4), dt, k2vx, 0.5);
    k3y = k1(x_cap(5), dt, k2vy, 0.5);
    k3z = k1(x_cap(6), dt, k2vz, 0.5);
    k3vx = k1v(x_cap(4), dt, k2vx, 0.5, 1);
    k3vy = k1v(x_cap(5), dt, k2vy, 0.5, 0);
    k3vz = k1v(x_cap(6), dt, k2vz, 0.5, 0);
    k3b = 0;

    k4x = k1(x_cap(4), dt, k3vx, 1);
    k4y = k1(x_cap(5), dt, k3vy, 1);
    k4z = k1(x_cap(6), dt, k3vz, 1);
    k4vx = k1v(x_cap(4), dt, k3vx, 1, 1);
    k4vy = k1v(x_cap(5), dt, k3vy, 1, 0);
    k4vz = k1v(x_cap(6), dt, k3vz, 1, 0);
    k4b = 0;

    f = @(x) [x(1) + (1/6)*(k1x+2*k2x+2*k3x+k4x);
              x(2) + (1/6)*(k1y+2*k2y+2*k3y+k4y);
              x(3) + (1/6)*(k1z+2*k2z+2*k3z+k4z);
              x(4) + (1/6)*(k1vx+2*k2vx+2*k3vx+k4vx);
              x(5) + (1/6)*(k1vy+2*k2vy+2*k3vy+k4vy);
              x(6) + (1/6)*(k1vz+2*k2vz+2*k3vz+k4vz)
              x(7) + (1/6)*(k1b+2*k2b+2*k3b+k4b);
        ];

    x_bar = f(x_cap);
    
    B = A(x_cap);
    D = eye(7) + B*dt;
    P_bar = D*P_cap*D' + Qd;

    y_meas = measurements(k,:);
   
    y_bar = h(x_bar);
    J = H(x_cap);
    S = J*P_bar*J' + R;
    K = P_bar*J'/S;
    P_cap = (I-K*J)*P_bar;
    Y = y_meas - y_bar;
    x_cap = x_bar + 1.0*K*Y';
    estimated_states(:, k) = x_cap;
    estimated_errors(:, k) = (true_states(:, k) - x_cap);
    estimated_pcapp(:, k) = sqrt(diag(P_cap));
    estimated_pcapn(:, k) = -1*sqrt(diag(P_cap));
end

disp(1/x_cap(7));

% ---------------------------------------------------------------
% Plot all neccessary graphs - code available in Plot_temp.m file
% ---------------------------------------------------------------

figure;
plot(time, estimated_states(1,:), 'DisplayName', 'Estimated States');
hold on;
plot(time, x_traj, 'DisplayName', 'Trajectory');
hold off;
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; % Set the grid color to black
ax.GridAlpha = 1; % Adjust the transparency of the grid lines
legend show;
