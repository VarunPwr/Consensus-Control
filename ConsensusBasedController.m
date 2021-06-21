%% Controller based on consensus based method.
%ri denotest the state vector of drone i where i belongs to {1,2,3 ..n}
%The state vector includes the x, y and z cartesian coordinates and heading
%angle phi.
%% Initialization
clear all;close all;
r10 = [0; 0; 0; 0];%Initial position of drone 1
r20 = [1; 0; 0; 0];%Drone 2
r30 = [0; 1; 0; 0];%Drone 3
rt0 = [10; 10; 10; pi/2];%Target state
R1 = [0; 0; 0; 0];%Final relative position of drone.
R2 = [5*cos(2*pi/3); 5*sin(2*pi/3); 0; 2*pi/3];
R3 = [5*cos(4*pi/3); 5*sin(4*pi/3); 0; 4*pi/3];
L3 = [2,-1,-1;-1,2,-1;-1,-1,2];%Laplacian matrix of network.
B = diag([1 0 0]);
dt = 0.001;%1000Hz frequency
theta = 0.1%time delay due to during communication
%% Using euler approximation for solving the ODEs
r1 = r10;
r2 = r20;
r3 = r30;
rt = rt0;
v1 = zeros(4,1);
v2 = zeros(4,1);
v3 = zeros(4,1);
vt = [1;0;0;0];%Heading velocity
t = 0 ;
kp = 1;
S1 = [t transpose(r1) transpose(v1)];
S2 = [t transpose(r2) transpose(v2)];
S3 = [t transpose(r3) transpose(v3)];
St = [t transpose(rt) transpose(vt)];
while t < 10
    u = controller(t, [r1 r2 r3 rt], [v1 v2 v3 vt], [R1 R2 R3], L3, B, kp);
    v1 = Rot(r1(4))*u(:,1);
    v2 = Rot(r2(4))*u(:,2);
    v3 = Rot(r3(4))*u(:,3);
    dR = dt*[Rot(r1(4))*u(:,1), Rot(r2(4))*u(:,2), Rot(r3(4))*u(:,3), vt];
    r1 = r1 + dR(:,1);
    r2 = r2 + dR(:,2);
    r3 = r3 + dR(:,3);
    rt = rt + dR(:,4);
    r1(4) = max(min(r1(4),2*pi),-2*pi);
    r2(4) = max(min(r2(4),2*pi),-2*pi);
    r3(4) = max(min(r3(4),2*pi),-2*pi);
    rt(4) = max(min(rt(4),2*pi),-2*pi);
    t = t + dt;
    S1 = vertcat(S1, [t transpose(r1) transpose(v1)]);
    S2 = vertcat(S2, [t transpose(r2) transpose(v2)]);
    S3 = vertcat(S3, [t transpose(r3) transpose(v3)]);
    St = vertcat(St, [t transpose(rt) transpose(vt)]);
end
%% Plot X Coordinate
figure
plot(S1(:,1), S1(:,2),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,2),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,2),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), [R1(1) + St(:,2)], '--g','linewidth',2);
hold on
plot(St(:,1), [R2(1) + St(:,2)], '--y','linewidth',2);
hold on
plot(St(:,1), [R3(1) + St(:,2)], '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,2); S2(:,2); S3(:,2)]) 1.25*max([S1(:,2); S2(:,2); S3(:,2)])])
title('X Position of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Y Coordinate
figure
plot(S1(:,1), S1(:,3),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,3),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,3),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), [R1(2) + St(:,3)], '--g','linewidth',2);
hold on
plot(St(:,1), [R2(2) + St(:,3)], '--y','linewidth',2);
hold on
plot(St(:,1), [R3(2) + St(:,3)], '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,3); S2(:,3); S3(:,3)]) 1.25*max([S1(:,3); S2(:,3); S3(:,3)])])
title('Y Position of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Z Coordinate
figure
plot(S1(:,1), S1(:,4),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,4),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,4),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), [R1(3) + St(:,4)], '--g','linewidth',2);
hold on
plot(St(:,1), [R2(3) + St(:,4)], '--y','linewidth',2);
hold on
plot(St(:,1), [R3(3) + St(:,4)], '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,4); S2(:,4); S3(:,4)]) 1.25*max([S1(:,4); S2(:,4); S3(:,4)])])
title('Altitude of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Heading angle
figure
plot(S1(:,1), S1(:,5),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,5),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,5),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), [R1(4) + St(:,5)], '--g','linewidth',2);
hold on
plot(St(:,1), [R2(4) + St(:,5)], '--y','linewidth',2);
hold on
plot(St(:,1), [R3(4) + St(:,5)], '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,5); S2(:,5); S3(:,5)]) 1.25*max([S1(:,5); S2(:,5); S3(:,5)])])
title('Heading of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot X Velocity angle
figure
plot(S1(:,1), S1(:,6),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,6),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,6),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), St(:,6), '--g','linewidth',2);
hold on
plot(St(:,1), St(:,6), '--y','linewidth',2);
hold on
plot(St(:,1), St(:,6), '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,6); S2(:,6); S3(:,6)]) 1.25*max([S1(:,6); S2(:,6); S3(:,6)])])
title('X Velocity of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Y Velocity angle
figure
plot(S1(:,1), S1(:,7),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,7),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,7),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), St(:,7), '--g','linewidth',2);
hold on
plot(St(:,1), St(:,7), '--y','linewidth',2);
hold on
plot(St(:,1), St(:,7), '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,7); S2(:,7); S3(:,7)]) 1.25*max([S1(:,7); S2(:,7); S3(:,7)])])
title('Y Velocity of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Z Velocity angle
figure
plot(S1(:,1), S1(:,8),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,8),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,8),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), St(:,8), '--g','linewidth',2);
hold on
plot(St(:,1), St(:,8), '--y','linewidth',2);
hold on
plot(St(:,1), St(:,8), '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,8); S2(:,8); S3(:,8)]) 1.25*max([S1(:,8); S2(:,8); S3(:,8)])])
title('Z Velocity of Drones')
legend('Drone1','Drone2','Drone3');
hold off
%% Plot Yaw Velocity angle
figure
plot(S1(:,1), S1(:,9),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,9),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,9),'b','linewidth',1.5);
hold on
xL = get(gca,'XLim');
plot(St(:,1), St(:,9), '--g','linewidth',2);
hold on
plot(St(:,1), St(:,9), '--y','linewidth',2);
hold on
plot(St(:,1), St(:,9), '--b','linewidth',2);
hold on
ylim([ 1.1*min([S1(:,9); S2(:,9); S3(:,9)]) 1.25*max([S1(:,9); S2(:,9); S3(:,9)])])
title('Yaw Velocity of Drones')
legend('Drone1','Drone2','Drone3');
hold off

% [r1, r2, r3, rt] = [r1, r2, r3, rt] + 
function [U] = controller(t, r, v, R, L3, B, kp)
U= [];
DL = L3-diag(diag(L3));
    for i = 1:3
    u = Rot(-r(4,i))*(-v(:,1:3)*transpose(DL(i,:)) - kp*(r(:,1:3)-R(:,1:3))*transpose(L3(i,:)) + B(i,i)*v(:,4) - kp*(B(i,i)*(r(:,i)-R(:,i)-r(:,4))))./(L3(i,i) + B(i,i));
    U = horzcat(U, u);
    end
end
function y = Rot(phi)
y = [cos(phi), -sin(phi), 0, 0;
     sin(phi), cos(phi), 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
end