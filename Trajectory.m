% Used for Generating Ballistic Missile Trajectory

clc; clear all; close all;

g = 9.81;
dt = 0.01; 
tmax = 500;
N = tmax/dt; 

fid = fopen("Target_Trajectory.txt", 'w');

x = zeros(1,N);
y = zeros(1,N);
z = zeros(1,N);
vx = zeros(1,N);
vy = zeros(1,N);
vz = zeros(1,N);
time = zeros(1,N);
betaa = zeros(1,N);

% Initial conditions
x(1) = 85570.0;
y(1) = 67885.0;
z(1) = 74770.0;
vx(1) = 350.0;
vy(1) = -95.0;
vz(1) = -355.0;
beta = 10000;
time(1) = 0;
betaa(1) = 10000;

for i = 1:N-1
    betaa(i+1) = 10000;
    time(i+1) = time(i) + dt;
    rho = 0.1819*2*exp(-0.0001516584*(x(i)-11000));
    vt = sqrt(vx(i)^2 + vy(i)^2 + vz(i)^2);
    
    ax = -rho*vt*vx(i)/(2*beta) - g;
    ay = -rho*vt*vy(i)/(2*beta);
    az = -rho*vt*vz(i)/(2*beta);
    
    k1x = dt * vx(i);
    k1y = dt * vy(i);
    k1z = dt * vz(i);
    k1vx = dt * ax;
    k1vy = dt * ay;
    k1vz = dt * az;
    
    k2x = dt * (vx(i) + 0.5*k1vx);
    k2y = dt * (vy(i) + 0.5*k1vy);
    k2z = dt * (vz(i) + 0.5*k1vz);
    k2vx = dt * (-rho*vt*(vx(i)+0.5*k1vx)/(2*beta) - g);
    k2vy = dt * (-rho*vt*(vy(i)+0.5*k1vy)/(2*beta));
    k2vz = dt * (-rho*vt*(vz(i)+0.5*k1vz)/(2*beta));
    
    k3x = dt * (vx(i) + 0.5*k2vx);
    k3y = dt * (vy(i) + 0.5*k2vy);
    k3z = dt * (vz(i) + 0.5*k2vz);
    k3vx = dt * (-rho*vt*(vx(i)+0.5*k2vx)/(2*beta) - g);
    k3vy = dt * (-rho*vt*(vy(i)+0.5*k2vy)/(2*beta));
    k3vz = dt * (-rho*vt*(vz(i)+0.5*k2vz)/(2*beta));
    
    k4x = dt * (vx(i) + k3vx);
    k4y = dt * (vy(i) + k3vy);
    k4z = dt * (vz(i) + k3vz);
    k4vx = dt * (-rho*vt*(vx(i)+k3vx)/(2*beta) - g);
    k4vy = dt * (-rho*vt*(vy(i)+k3vy)/(2*beta));
    k4vz = dt * (-rho*vt*(vz(i)+k3vz)/(2*beta));
    
    x(i+1) = x(i) + (1/6)*(k1x + 2*k2x + 2*k3x + k4x);
    y(i+1) = y(i) + (1/6)*(k1y + 2*k2y + 2*k3y + k4y);
    z(i+1) = z(i) + (1/6)*(k1z + 2*k2z + 2*k3z + k4z);
    vx(i+1) = vx(i) + (1/6)*(k1vx + 2*k2vx + 2*k3vx + k4vx);
    vy(i+1) = vy(i) + (1/6)*(k1vy + 2*k2vy + 2*k3vy + k4vy);
    vz(i+1) = vz(i) + (1/6)*(k1vz + 2*k2vz + 2*k3vz + k4vz);
    
    fprintf(fid,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', time(i), x(i), y(i), z(i), vx(i), vy(i), vz(i), betaa(i));

    if x(i+1) <= 0
        disp(i+1);
        break; 
    end
end

fprintf(fid,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', time(i+1), x(i+1), y(i+1), z(i+1), vx(i+1), vy(i+1), vz(i+1), betaa(i+1));
fclose(fid);

x = x(1:i+1);
y = y(1:i+1);
z = z(1:i+1);
time = time(1:i+1);
betaa = betaa(1:i+1);

figure;
plot3(z, y, x);
xlabel('Z');
ylabel('Y');
zlabel('X');
title('3D Trajectory of Ballistic Target');
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridColor = [0, 0, 0]; 
ax.GridAlpha = 0.5; 

figure;
plot(time, x);
xlabel("Time");
ylabel("Height");
title("Height vs Time Plot");
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridColor = [0, 0, 0]; 
ax.GridAlpha = 1; 

