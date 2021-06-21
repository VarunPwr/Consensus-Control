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
dt = 0.0005;%1000Hz frequency
theta = 0.00001;%time delay due to during communication
%% Using euler approximation for solving the ODEs
r1 = r10;
r2 = r20;
r3 = r30;
rt = rt0;
v1 = zeros(4,1);
v2 = zeros(4,1);
v3 = zeros(4,1);
vt = ((alpha(0))^1.5)*[1;0;0;0.25];%Heading velocity
t = 0 ;
kp = 7.5;
S1 = [t transpose(r1) transpose(v1)];%True state vector
S2 = [t transpose(r2) transpose(v2)];
S3 = [t transpose(r3) transpose(v3)];
St = [t transpose(rt) transpose(vt)];
while t < 25
    %% Drone 1
    %rij is vector i viewed from j. Note that the network is strongly
    %connected and target position is only viewed by drone1
    vt = ((alpha(t))^1.5)*[sin(pi*0.25*t);cos(pi*0.25*t);0;0.25];%Heading velocity    
    i =1;
%     if t > theta
        r11 = [S1(end,2); S1(end,3); S1(end,4); S1(end,5)];
        v11 = [S1(end,6); S1(end,7); S1(end,8); S1(end,9)];
        r21 = [S2(end,2); S2(end,3); S2(end,4); S2(end,5)];
        v21 = [S2(end,6); S2(end,7); S2(end,8); S2(end,9)];
        r31 = [S3(end,2); S3(end,3); S3(end,4); S3(end,5)];
        v31 = [S3(end,6); S3(end,7); S3(end,8); S3(end,9)];
        rt1 = [St(end,2); St(end,3); St(end,4); St(end,5)];
        vt1 = [St(end,6); St(end,7); St(end,8); St(end,9)];
%         r21 = [interp1(S2(:,1),S2(:,2),t-theta); interp1(S2(:,1),S2(:,3),t-theta); interp1(S2(:,1),S2(:,4),t-theta); interp1(S2(:,1),S2(:,5),t-theta)];
%         r31 = [interp1(S3(:,1),S3(:,2),t-theta); interp1(S3(:,1),S3(:,3),t-theta); interp1(S3(:,1),S3(:,4),t-theta); interp1(S3(:,1),S3(:,5),t-theta)];
%         rt1 = [interp1(St(:,1),St(:,2),t-theta); interp1(St(:,1),St(:,3),t-theta); interp1(St(:,1),St(:,4),t-theta); interp1(St(:,1),St(:,5),t-theta)];
%         v21 = [interp1(S2(:,1),S2(:,6),t-theta); interp1(S2(:,1),S2(:,7),t-theta); interp1(S2(:,1),S2(:,8),t-theta); interp1(S2(:,1),S2(:,9),t-theta)];
%         v31 = [interp1(S3(:,1),S3(:,6),t-theta); interp1(S3(:,1),S3(:,7),t-theta); interp1(S3(:,1),S3(:,8),t-theta); interp1(S3(:,1),S3(:,9),t-theta)];
%         vt1 = [interp1(St(:,1),St(:,6),t-theta); interp1(St(:,1),St(:,7),t-theta); interp1(St(:,1),St(:,8),t-theta); interp1(St(:,1),St(:,9),t-theta)];
%     else
%         r11 = [S1(end,2); S1(end,3); S1(end,4); S1(end,5)];
%         v11 = [S1(end,6); S1(end,7); S1(end,8); S1(end,9)];
%         r21 = zeros(4,1);
%         r31 = zeros(4,1);
%         rt1 = zeros(4,1);
%         v21 = zeros(4,1);
%         v31 = zeros(4,1);
%         vt1 = zeros(4,1);
%     end
    r21 = awgn(r21, 10);
    r31 = awgn(r31, 10);
    rt1 = awgn(rt1, 10);
    v21 = awgn(v21, 10);
    v31 = awgn(v31, 10);
    vt1 = awgn(vt1, 10);
    u1 = controller(t, [r11 r21 r31 rt1], [v11 v21 v31 vt1], [R1 R2 R3], L3, B, kp,i);
    v11 = Rot(r11(4))*u1;
    dR1 = dt*Rot(r11(4))*u1;
    r11 = r11 + dR1;
%     r11(4) = max(min(r11(4),2*pi),-2*pi);
    S1 = vertcat(S1, [t + dt transpose(r11) transpose(v11)]);
        %% Drone 2
    %rij is vector i viewed from j. Note that the network is strongly
    %connected and target position is only viewed by drone1 
    i=2;
%     if t > theta
        r22 = [S2(end,2); S2(end,3); S2(end,4); S2(end,5)];
        v22 = [S2(end,6); S2(end,7); S2(end,8); S2(end,9)];
        r12 = [S1(end,2); S1(end,3); S1(end,4); S1(end,5)];
        v12 = [S1(end,6); S1(end,7); S1(end,8); S1(end,9)];
        r32 = [S3(end,2); S3(end,3); S3(end,4); S3(end,5)];
        v32 = [S3(end,6); S3(end,7); S3(end,8); S3(end,9)];
        rt2 = [St(end,2); St(end,3); St(end,4); St(end,5)];
        vt2 = [St(end,6); St(end,7); St(end,8); St(end,9)];
%         r12 = [interp1(S1(:,1),S1(:,2),t-theta); interp1(S1(:,1),S1(:,3),t-theta); interp1(S1(:,1),S1(:,4),t-theta); interp1(S1(:,1),S1(:,5),t-theta)];
%         r32 = [interp1(S3(:,1),S3(:,2),t-theta); interp1(S3(:,1),S3(:,3),t-theta); interp1(S3(:,1),S3(:,4),t-theta); interp1(S3(:,1),S3(:,5),t-theta)];
%         rt2 = [interp1(St(:,1),St(:,2),t-2*theta); interp1(St(:,1),St(:,3),t-2*theta); interp1(St(:,1),St(:,4),t-2*theta); interp1(St(:,1),St(:,5),t-2*theta)];
%         v12 = [interp1(S1(:,1),S1(:,6),t-theta); interp1(S1(:,1),S1(:,7),t-theta); interp1(S1(:,1),S1(:,8),t-theta); interp1(S1(:,1),S1(:,9),t-theta)];
%         v32 = [interp1(S3(:,1),S3(:,6),t-theta); interp1(S3(:,1),S3(:,7),t-theta); interp1(S3(:,1),S3(:,8),t-theta); interp1(S3(:,1),S3(:,9),t-theta)];
%         vt2 = [interp1(St(:,1),St(:,6),t-2*theta); interp1(St(:,1),St(:,7),t-2*theta); interp1(St(:,1),St(:,8),t-2*theta); interp1(St(:,1),St(:,9),t-2*theta)];
% %     else
%         r12 = zeros(4,1);
%         r22 = [S2(end,2); S2(end,3); S2(end,4); S2(end,5)];
%         r32 = zeros(4,1);
%         rt2 = zeros(4,1);
%         v22 = [S2(end,6); S2(end,7); S2(end,8); S2(end,9)];
%         v12 = zeros(4,1);
%         v32 = zeros(4,1);
%         vt2 = zeros(4,1);
%     end
    r12 = awgn(r12, 10);
    r32 = awgn(r32, 10);
    rt2 = awgn(rt2, 10);
    v12 = awgn(v12, 10);
    v32 = awgn(v32, 10);
    vt2 = awgn(vt2, 10);
    u2 = controller(t, [r12 r22 r32 rt2], [v12 v22 v32 vt2], [R1 R2 R3], L3, B, kp,i);
    v22 = Rot(r22(4))*u2;
    dR2 = dt*Rot(r22(4))*u2;
    r22 = r22 + dR2;
%     r22(4) = max(min(r22(4),2*pi),-2*pi);
    S2 = vertcat(S2, [t + dt transpose(r22) transpose(v22)]);
         %% Drone 3
    %rij is vector i viewed from j. Note that the network is strongly
    %connected and target position is only viewed by drone1 
    i=3;
%     if t > theta
        r33 = [S3(end,2); S3(end,3); S3(end,4); S3(end,5)];
        v33 = [S3(end,6); S3(end,7); S3(end,8); S3(end,9)];
        r13 = [S1(end,2); S1(end,3); S1(end,4); S1(end,5)];
        v13 = [S1(end,6); S1(end,7); S1(end,8); S1(end,9)];
        r23 = [S2(end,2); S2(end,3); S2(end,4); S2(end,5)];
        v23 = [S2(end,6); S2(end,7); S2(end,8); S2(end,9)];
        rt3 = [St(end,2); St(end,3); St(end,4); St(end,5)];
        vt3 = [St(end,6); St(end,7); St(end,8); St(end,9)];
%         r13 = [interp1(S1(:,1),S1(:,2),t-theta); interp1(S1(:,1),S1(:,3),t-theta); interp1(S1(:,1),S1(:,4),t-theta); interp1(S1(:,1),S1(:,5),t-theta)];
%         r23 = [interp1(S2(:,1),S2(:,2),t-theta); interp1(S2(:,1),S2(:,3),t-theta); interp1(S2(:,1),S2(:,4),t-theta); interp1(S2(:,1),S2(:,5),t-theta)];
%         rt3 = [interp1(St(:,1),St(:,2),t-2*theta); interp1(St(:,1),St(:,3),t-2*theta); interp1(St(:,1),St(:,4),t-2*theta); interp1(St(:,1),St(:,5),t-2*theta)];
%         v13 = [interp1(S1(:,1),S1(:,6),t-theta); interp1(S1(:,1),S1(:,7),t-theta); interp1(S1(:,1),S1(:,8),t-theta); interp1(S1(:,1),S1(:,9),t-theta)];
%         v23 = [interp1(S2(:,1),S2(:,6),t-theta); interp1(S2(:,1),S2(:,7),t-theta); interp1(S2(:,1),S2(:,8),t-theta); interp1(S2(:,1),S2(:,9),t-theta)];
%         vt3 = [interp1(St(:,1),St(:,6),t-2*theta); interp1(St(:,1),St(:,7),t-2*theta); interp1(St(:,1),St(:,8),t-2*theta); interp1(St(:,1),St(:,9),t-2*theta)];
%     else
%         r33 = [S3(end,2); S3(end,3); S3(end,4); S3(end,5)];
%         v33 = [S3(end,6); S3(end,7); S3(end,8); S3(end,9)];
%         r13 = zeros(4,1);
%         r23 = zeros(4,1);
%         rt3 = zeros(4,1);
%         v13 = zeros(4,1);
%         v23 = zeros(4,1);
%         vt3 = zeros(4,1);
%     end
    
    r13 = awgn(r13, 10);
    r23 = awgn(r23, 10);
    rt3 = awgn(rt3, 10);
    v13 = awgn(v13, 10);
    v23 = awgn(v23, 10);
    vt3 = awgn(vt3, 10);
    u3 = controller(t, [r13 r23 r33 rt3], [v13 v23 v33 vt3], [R1 R2 R3], L3, B, kp,i);
    v33 = Rot(r33(4))*u3;
    dR3 = dt*Rot(r33(4))*u3;
    r33 = r33 + dR3;
%     r33(4) = max(min(r33(4),2*pi),-2*pi);
    S3 = vertcat(S3, [t + dt transpose(r33) transpose(v33)]);
    
    %%
    rt = rt + dt*vt;
    rt(4) = max(min(rt(4),2*pi),-2*pi);
    St = vertcat(St, [t + dt transpose(rt) transpose(vt)]);
    t = t + dt;
end
%% Simulation Plot
% figure
% for i = 1 : length(S1)
%     plot(S1(i,2), S1(i,3),'o')
%     hold on 
%     Z = Rot(S1(i,5))*[-1,1,0,0 ;-1,-1,1,0 ;0,0,0,0 ;0,0,0,0];
%     plot([S1(i,2)+Z(1,1), S1(i,2)+Z(1,2)],[S1(i,3)+Z(2,1), S1(i,3)+Z(2,2)],'b')
%     hold on
%     plot([S1(i,2)+Z(1,2), S1(i,2)+Z(1,3)],[S1(i,3)+Z(2,2), S1(i,3)+Z(2,3)],'b')
%     hold on
%     plot([S1(i,2)+Z(1,3), S1(i,2)+Z(1,1)],[S1(i,3)+Z(2,3), S1(i,3)+Z(2,1)],'b')
%     hold on
%     plot([S1(i,2)+Z(1,3), S1(i,2)+Z(1,4)],[S1(i,3)+Z(2,3), S1(i,3)+Z(2,4)],'b')
%     hold on
%     %%%%%%%%%%%%%%%
%     plot(S2(i,2), S2(i,3),'o')
%     hold on 
%     Z = Rot(S2(i,5))*[-1,1,0,0 ;-1,-1,1,0 ;0,0,0,0 ;0,0,0,0];
%     plot([S2(i,2)+Z(1,1), S2(i,2)+Z(1,2)],[S2(i,3)+Z(2,1), S2(i,3)+Z(2,2)],'r')
%     hold on
%     plot([S2(i,2)+Z(1,2), S2(i,2)+Z(1,3)],[S2(i,3)+Z(2,2), S2(i,3)+Z(2,3)],'r')
%     hold on
%     plot([S2(i,2)+Z(1,3), S2(i,2)+Z(1,1)],[S2(i,3)+Z(2,3), S2(i,3)+Z(2,1)],'r')
%     hold on
%     plot([S2(i,2)+Z(1,3), S2(i,2)+Z(1,4)],[S2(i,3)+Z(2,3), S2(i,3)+Z(2,4)],'r')
%     hold on
%     %%%%%%%%%%%%%%%
%     plot(S3(i,2), S3(i,3),'o')
%     hold on 
%     Z = Rot(S3(i,5))*[-1,1,0,0 ;-1,-1,1,0 ;0,0,0,0 ;0,0,0,0];
%     plot([S3(i,2)+Z(1,1), S3(i,2)+Z(1,2)],[S3(i,3)+Z(2,1), S3(i,3)+Z(2,2)],'g')
%     hold on
%     plot([S3(i,2)+Z(1,2), S3(i,2)+Z(1,3)],[S3(i,3)+Z(2,2), S3(i,3)+Z(2,3)],'g')
%     hold on
%     plot([S3(i,2)+Z(1,3), S3(i,2)+Z(1,1)],[S3(i,3)+Z(2,3), S3(i,3)+Z(2,1)],'g')
%     hold on
%     plot([S3(i,2)+Z(1,3), S3(i,2)+Z(1,4)],[S3(i,3)+Z(2,3), S3(i,3)+Z(2,4)],'g')
%     hold on
%      %%%%%%%%%%%%%%%
%     plot(St(i,2), St(i,3),'o')
%     hold on 
%     Z = Rot(St(i,5))*[-1,1,0,0 ;-1,-1,1,0 ;0,0,0,0 ;0,0,0,0];
%     plot([St(i,2)+Z(1,1), St(i,2)+Z(1,2)],[St(i,3)+Z(2,1), St(i,3)+Z(2,2)],'k')
%     hold on
%     plot([St(i,2)+Z(1,2), St(i,2)+Z(1,3)],[St(i,3)+Z(2,2), St(i,3)+Z(2,3)],'k')
%     hold on
%     plot([St(i,2)+Z(1,3), St(i,2)+Z(1,1)],[St(i,3)+Z(2,3), St(i,3)+Z(2,1)],'k')
%     hold on
%     plot([St(i,2)+Z(1,3), St(i,2)+Z(1,4)],[St(i,3)+Z(2,3), St(i,3)+Z(2,4)],'k')
%     xlim([-20 20])
%     ylim([-20 20])
%     pause(0.01*dt)
%     hold off  
% end
%% Plot X Coordinate
figure
plot(S1(:,1), S1(:,2),'g','linewidth',1.5);
hold on
plot(S2(:,1), S2(:,2),'y','linewidth',1.5);
hold on
plot(S3(:,1), S3(:,2),'b','linewidth',1.5);
hold on
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
%%
sigma1 = sum(var(S1(:,2:end)))
mu1 = sum(mean(S1(:,2:end)))
sigma2 = sum(var(S2(:,2:end)))
mu2 = sum(mean(S2(:,2:end)))
sigma3 = sum(var(S3(:,2:end)))
mu3 = sum(mean(S3(:,2:end)))
sigmat = sum(var(St(:,2:end)))
mut = sum(mean(St(:,2:end)))
%%
% [r1, r2, r3, rt] = [r1, r2, r3, rt] + 
function u = controller(t, r, v, R, L3, B, kp, i)
DL = L3-diag(diag(L3));
u = Rot(-r(4,i))*(-v(:,1:3)*transpose(DL(i,:)) - kp*alpha(t)*(r(:,1:3)-R(:,1:3))*transpose(L3(i,:)) + B(i,i)*v(:,4) - kp*alpha(t)*(B(i,i)*(r(:,i)-R(:,i)-r(:,4))));
end
function y = Rot(phi)
y = [cos(phi), -sin(phi), 0, 0;
     sin(phi), cos(phi), 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
end
function y = alpha(k)
p = 0.51;
c = 4;
u = 1;
if k > u
    y = c/k^p;
else
    y = c*(-(p+1)/(u)^(p+2)*k^2 + (2 + p)/(u^(p+1))*k);
end
end