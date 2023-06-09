% Script that plots the results of the grasping force experiment. 
clear
load('../data/ag_h_1.mat');

%% Torque

% Compute estimate forces
kinetostatic_model_PTV;

% Compute torque at the actuator
tau_ideal = pwm2torque(pwm_openloop);
act_tau = tau_a_;





%% Horizontal forces

hforce_center = forces_horizontal(:,1);
hsensor = fsensor;
htime = time;

p1 = 0.02;
p2 = 0.025;

kinetostatic_model_PTV;
hforce_up =  forces_horizontal(:,1);

p1 = 0.02;
p2 = 0.015;
 
kinetostatic_model_PTV;
hforce_down =  forces_horizontal(:,1);
 
%% Vertical forces
clearvars -except tau_ideal act_tau htime hforce_up hsensor hforce_center hforce_down

% Load new data and compute forces
load('../data/ag_v_1.mat');
kinetostatic_model_PTV;

vtime = time;
vsensor = fsensor;
vforce_center = forces_vertical(:,1);

p1 = 0.02;
p2 = 0.025;

kinetostatic_model_PTV;
vforce_up =  forces_vertical(:,1);

p1 = 0.02;
p2 = 0.015;

kinetostatic_model_PTV;
vforce_down =  forces_vertical(:,1);
 
clearvars -except tau_ideal act_tau htime hforce_up hforce_down hforce_center hsensor vtime vsensor vforce_center vforce_up vforce_down

%% Plots

% Some useful variables

msensor = 0; % peso del sensor de fuerza en N

color_sensor = [0.42,0.24,0.02];
color_estim = [1,0.45,0.1];
color_area = [1,0.56,0.18];
linew = 1;

t_off = -0.25;
time_interval = [25, 41];

fontsize = 16;

% Torque
figure, 
hold on, grid, grid minor, 
plot(htime-time_interval(1), act_tau(1,:), 'Color',[0,0.7843,1]);
plot(htime-time_interval(1), act_tau(2,:), 'Color',[1,0,0]);
plot(htime-time_interval(1), act_tau(3,:), 'Color',[0.2,1,0]);
plot(htime-time_interval(1), act_tau(4,:), 'Color',[1,0,0.9686]);
plot(htime-time_interval(1), tau_ideal, 'Color', [9,0,135]/255);
xlim([0, time_interval(2)-time_interval(1)]); 
ax = gca;
ax.FontSize = fontsize
xlabel('Time [s]','FontSize',fontsize); 
ylabel('Actuation Torque [Nm]','FontSize',fontsize)
legend('\tau_{a1}','\tau_{a2}','\tau_{a3}','\tau_{a4}','\tau_{PWM}','FontSize',fontsize);
saveas(gcf,'torque.pdf');

% H-forces
hx = htime(1:10:end)-time_interval(1);
figure, 
hold on, grid, grid minor, 
plot(hx+t_off, hsensor(1:10:end), 'Color',color_sensor, 'LineWidth',linew);
plot(hx, hforce_center(1:10:end), 'Color',color_estim, 'LineWidth',linew);
patch([hx';flipud(hx')], [hforce_up(1:10:end); flipud(hforce_down(1:10:end))], color_area, 'EdgeColor','none', 'FaceAlpha',0.2)
% plot(hx, hforce_up(1:10:end), 'Color',color_real);
% plot(hx, hforce_down(1:10:end), 'Color',color_real);
xlim([0, time_interval(2)-time_interval(1)]); 
ylim([0,27]);
ax = gca;
ax.FontSize = fontsize;
xlabel('Time [s]','FontSize',fontsize); 
ylabel('Horizontal Force [N]','FontSize',fontsize)
legend('Ground-truth Force','Measured Force','FontSize',fontsize);
saveas(gcf,'hforce.pdf');

% V-forces
vx = vtime(1:10:end)-time_interval(1);
figure, 
hold on, grid, grid minor, 
plot(vx +t_off, vsensor(1:10:end), 'Color',color_sensor, 'LineWidth',linew);
plot(vx, vforce_center(1:10:end)-msensor, 'Color',color_estim, 'LineWidth',linew);
patch([vx';flipud(vx')], [vforce_up(1:10:end); flipud(vforce_down(1:10:end))], color_area, 'EdgeColor','none', 'FaceAlpha',0.2)
% plot(vx, vforce_up(1:10:end), 'Color',color_area);
% plot(vx, vforce_down(1:10:end), 'Color',color_area);
xlim([0, time_interval(2)-time_interval(1)]); 
ylim([0,15]);
ax = gca
ax.FontSize = fontsize;
xlabel('Time [s]','FontSize',fontsize); 
ylabel('Vertical Force [N]','FontSize',fontsize)
legend('Ground-truth Force','Measured Force', 'FontSize',fontsize);
saveas(gcf,'vforce.pdf');

efh = sqrt((hsensor(600+25:10:6000)'-hforce_center(600:10:6000-25)).^2);
efv = sqrt((vsensor(600+25:10:6000)'-vforce_center(600:10:6000-25)).^2);

[mean(efh), var(efh)]
[mean(efv), var(efv)]
