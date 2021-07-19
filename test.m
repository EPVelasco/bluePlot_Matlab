close all;clear;clc
figure; 
subplot(3,2,1)
axis equal; 
grid on; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
bluePlot(0,0,0,0,0.4,0.001)

subplot(3,2,3)
view(90,0)
axis equal; 
grid on; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
camlight('rigth');
bluePlot(0,0,0,0,0.4,0.001)

subplot(3,2,5)
view(90,0)
axis equal; 
grid on; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
camlight('rigth');
bluePlot(0,0,0,pi/2,0.4,0.001)

subplot(1,2,2)
view(45,45)
axis equal; 
grid on; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
camlight('HEADLIGHT');
bluePlot(0,0,0,0,0.4,0.001)


