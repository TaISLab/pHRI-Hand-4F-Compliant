% seguimiento en Y 
load('../data/complY5.mat');
figure,
subplot(2,1,1),
plot(time-14,franka_force(2,:)), hold on,
ylabel('Force [N]'),%xlabel('Time [s]'),
grid, hold on,
plot(time-14,-interaction_force(1,:)),
    % For drawing the legend
plot(time(1:2)-14,franka_pose(2,1:2),'Color',[.5,.5,0]);
plot(time(1:2)-14,franka_velocity(2,1:2),'Color',[.5,0,.5]);
legend('Ground-truth', 'Estimated','Robot End-effector Position','Robot End-effector Velocity','Robot Measured Force', 'Location','northwest')
xlim([0,20]),
% Second subplot
subplot(2,1,2),
yyaxis left, ax = gca; ax.YColor = [.5,.5,0];
plot(time-14, franka_pose(2,:),'Color',[.5,.5,0]), 
ylabel('Position [m]'),xlabel('Time [s]'),
yyaxis right, ax = gca; ax.YColor = [.5,0,.5];
plot(time-14, franka_velocity(2,:),'Color',[.5,0,.5]), 
grid on, xlim([0,20]),
ylabel('Velocity [m/s]'),
title('Active compliance along X')


% seguimiento en rot z
load('../data/complT3.mat');
ang = quat2eul([franka_pose(7,:);franka_pose(4:6,:)]','XYZ');
figure,
subplot(2,1,1),
plot(time-9.5,franka_force(2,:)), hold on,
ylabel('Torque [Nm]'),%xlabel('Time [s]'),
grid, hold on,
plot(time-9.5,-interaction_force(1,:)),
    % For drawing the legend
plot(time(1:2)-9.5,franka_pose(2,1:2),'Color',[.5,.5,0]);
plot(time(1:2)-9.5,franka_velocity(2,1:2),'Color',[.5,0,.5]);
xlim([0,15]),
% Second subplot
subplot(2,1,2),
yyaxis left, ax = gca; ax.YColor = [.5,.5,0];
plot(time-9.5, ang(:,3),'Color',[.5,.5,0]), 
ylim([-0.6,0.6]),
ylabel('Position [rad]'),xlabel('Time [s]'),
yyaxis right, ax = gca; ax.YColor = [.5,0,.5];
plot(time-9.5, franka_velocity(6,:),'Color',[.5,0,.5]), 
grid on, ylim([-0.75,0.75])
ylabel('Velocity [rad/s]'),
xlim([0,15]);
title('Active compliance around Z')


% estatico Y
load('../data/intFy_PERFECTO.mat');
figure, 
plot(time-15.7,-franka_force(2,:));
hold on, grid on, 
plot(time-15.7, interaction_force(1,:));
xlim([0,11.1]); ylabel('Force [N]'), xlabel('Time [s]'),
legend('Ground-truth', 'Estimated','Location','southwest')
error = interaction_force(1,1572:2682) + franka_force(2,1572:2682);
RMSE = sqrt(mean(error.^2)) 
var(sqrt(error.^2))
MAE = mean(abs(error))
var(abs(error))
title('Interaction Force in X')

% estatico Z
load('../data/interZ4.mat');
off = 11;
figure, 
plot(time-off,franka_force(3,:)+1.75);
hold on, grid on, 
plot(time-off, interaction_force(2,:));
 xlim([0,13.7]); ylabel('Force [N]'), xlabel('Time [s]'),
% legend('Ground-truth', 'Estimated')
title('Interaction Force in Z')

% estatico TZ
load('../data/interT7.mat');
off = 9;
figure, 
plot(time-off,-franka_force(6,:));
hold on, grid on, 
plot(time-off, interaction_force(3,:));
 xlim([0,15]); ylabel('Torque [Nm]'), xlabel('Time [s]'),
% legend('Ground-truth', 'Estimated')
error = interaction_force(3,902:2402) + franka_force(6,902:2402);
RMSE = sqrt(mean(error.^2)) 
var(sqrt(error.^2))
MAE = mean(abs(error))
var(abs(error))
title('Interaction torque around Z')
