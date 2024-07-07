% Target Trajectory Generation for Maneuvering Target

clc; clear all; close all;

g = 9.81;
dt = 0.01; 
tmax = 500;
N = tmax/dt; 

fid = fopen("Target_Trajectory_Man.txt", 'w');

x = zeros(1,N);
y = zeros(1,N);
z = zeros(1,N);
vx = zeros(1,N);
vy = zeros(1,N);
vz = zeros(1,N);
time = zeros(1,N);

x(1) = 100000.0; y(1) = 10000.0; z(1) = 1000000.0;
vx(1) = 100.0; vy(1) = -25.0; vz(1) = -2000.0;
beta = 1/15000; z1 = 1e-6; z2 = -1*1e-4;
time(1) = 0;

for i = 1:N-1
    time(i+1) = time(i) + dt;
    
    k1 = @(x, delt, k, n) (delt*(x+(n*k)));
    k1v = @(delt, n, k1, k2, k3, k4) delt*(0.5*(0.1819*2*exp(-0.0001516584*((x(i)+n*k4)-11000)))*(sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2))*((-1*(vx(i)+n*k1)*beta)-(z2*sqrt((vy(i)+n*k2)^2+(vz(i)+n*k3)^2)))-g);
    k2v = @(delt, n, k1, k2, k3, k4) delt*0.5*(0.1819*2*exp(-0.0001516584*((x(i)+n*k4)-11000)))*((sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2))^2)*((-1*(vy(i)+n*k2)*beta/(sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2)))+((vz(i)+n*k3)*z1/sqrt((vy(i)+n*k2)^2+(vz(i)+n*k3)^2))+((vx(i)+n*k1)*(vy(i)+n*k2)*z2/((sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2))*sqrt((vy(i)+n*k2)^2+(vz(i)+n*k3)^2))));
    k3v = @(delt, n, k1, k2, k3, k4) delt*0.5*(0.1819*2*exp(-0.0001516584*((x(i)+n*k4)-11000)))*((sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2))^2)*((-1*(vz(i)+n*k3)*beta/(sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2)))-((vy(i)+n*k2)*z1/sqrt((vy(i)+n*k2)^2+(vz(i)+n*k3)^2))+((vx(i)+n*k1)*(vz(i)+n*k3)*z2/((sqrt((vx(i)+n*k1)^2 + (vy(i)+n*k2)^2 + (vz(i)+n*k3)^2))*sqrt((vy(i)+n*k2)^2+(vz(i)+n*k3)^2))));

    k1x = k1(vx(i),dt,0,0);
    k1y = k1(vy(i),dt,0,0);
    k1z = k1(vz(i),dt,0,0);
    k1vx = k1v(dt,0,0,0,0,0);
    k1vy = k2v(dt,0,0,0,0,0);
    k1vz = k3v(dt,0,0,0,0,0);

    k2x = k1(vx(i),dt,k1vx,0.5);
    k2y = k1(vy(i),dt,k1vy,0.5);
    k2z = k1(vz(i),dt,k1vz,0.5);
    k2vx = k1v(dt,0.5,k1vx,k1vy,k1vz,k1x);
    k2vy = k2v(dt,0.5,k1vx,k1vy,k1vz,k1x);
    k2vz = k3v(dt,0.5,k1vx,k1vy,k1vz,k1x);

    k3x = k1(vx(i),dt,k2vx,0.5);
    k3y = k1(vy(i),dt,k2vy,0.5);
    k3z = k1(vz(i),dt,k2vz,0.5);
    k3vx = k1v(dt,0.5,k2vx,k2vy,k2vz,k2x);
    k3vy = k2v(dt,0.5,k2vx,k2vy,k2vz,k2x);
    k3vz = k3v(dt,0.5,k2vx,k2vy,k2vz,k2x);

    k4x = k1(vx(i),dt,k3vx,1);
    k4y = k1(vy(i),dt,k3vy,1);
    k4z = k1(vz(i),dt,k3vz,1);
    k4vx = k1v(dt,1,k3vx,k3vy,k3vz,k3x);
    k4vy = k2v(dt,1,k3vx,k3vy,k3vz,k3x);
    k4vz = k3v(dt,1,k3vx,k3vy,k3vz,k3x);

    x(i+1) = x(i) + (1/6)*(k1x + 2*k2x + 2*k3x + k4x);
    y(i+1) = y(i) + (1/6)*(k1y + 2*k2y + 2*k3y + k4y);
    z(i+1) = z(i) + (1/6)*(k1z + 2*k2z + 2*k3z + k4z);
    vx(i+1) = vx(i) + (1/6)*(k1vx + 2*k2vx + 2*k3vx + k4vx);
    vy(i+1) = vy(i) + (1/6)*(k1vy + 2*k2vy + 2*k3vy + k4vy);
    vz(i+1) = vz(i) + (1/6)*(k1vz + 2*k2vz + 2*k3vz + k4vz);

    fprintf(fid,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n', time(i), x(i), y(i), z(i), vx(i), vy(i), vz(i), beta, z1, z2);

    if x(i+1) <= 0
        disp(i+1);
        break; 
    end
end

fprintf(fid,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n', time(i+1), x(i+1), y(i+1), z(i+1), vx(i+1), vy(i+1), vz(i+1), beta, z1, z2);
fclose(fid);

x = x(1:i+1);
y = y(1:i+1);
z = z(1:i+1);
time = time(1:i+1);

figure;
plot3(z, y, x);
xlabel('Z');
ylabel('Y');
zlabel('X');
title('3D Trajectory of Ballistic Target');
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; 
ax.GridAlpha = 0.5; 

figure;
plot(time, x);
xlabel("Time");
ylabel("Height");
title("Height vs Time Plot");
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridColor = [0, 0, 0]; 
ax.GridAlpha = 1; 