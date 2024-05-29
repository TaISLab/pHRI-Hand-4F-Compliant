figure, plot(time, -franka_force(2,:)), hold on, plot(time, interaction_force(1,:), 'LineStyle','--')
figure, plot(time, franka_force(3,:)+1.75), hold on, plot(time, interaction_force(2,:)-1.3, 'LineStyle','--')
figure, plot(time, -franka_force(6,:)), hold on, plot(time, interaction_force(3,:), 'LineStyle','--')

Fht = zeros(1,length(theta_a_0));
Fvt = zeros(1,length(theta_a_0));
Tst = zeros(1,length(theta_a_0));

for j = 1:length(theta_a_0)
    theta_a = [theta_a_0(j);theta_a_1(j);theta_a_2(j);theta_a_3(j)];
    theta_1 = [theta_1_0(j);theta_1_1(j);theta_1_2(j);theta_1_3(j)];
    theta_2 = [theta_2_0(j);theta_2_1(j);theta_2_2(j);theta_2_3(j)];

    [grasp_force, int_h_force, int_v_force, int_tau_z] = kinetostaticModel (theta_a, theta_1, theta_2, 0.02, 0.02);
    Fht(j) = int_h_force;
    Fvt(j) = int_v_force;
    Tst(j) = int_tau_z;

end

figure, plot([franka_force(2,:);Fht]')
figure, plot([franka_force(3,:);Fvt]')
figure, plot([franka_force(6,:);Tst]')

figure, plot([franka_force(2,:);forces_horizontal(:,1)']')
figure, plot([franka_force(3,:);-forces_vertical(:,1)']')
figure, plot([-franka_force(6,:);int_tau_z(:,1)']')

figure, plot(franka_force(2,:)), hold on, plot(0.5*[Fxd11(:,1)+Fxd12(:,1)+Fxd41(:,1)+Fxd42(:,1), Fxd21(:,1)+Fxd22(:,1) + Fxd31(:,1)+Fxd32(:,1)])
figure, plot(franka_force(2,:)), hold on, plot([Fxd11(:,1)+Fxd12(:,1)+Fxd41(:,1)+Fxd42(:,1) + Fxd21(:,1)+Fxd22(:,1) + Fxd31(:,1)+Fxd32(:,1)])



e_y = (franka_force(2,1030:2600)-interaction_force(1,1030:2600));
rmse_y = sqrt(mean(e_y.^2))
sigma1_y = var(e_y)
mae_y = mean(abs(e_y))
sigma2_y = var(e_y)

% R45 = [0.7071 -0.7071 0; 0.7071 0.7071 0; 0 0 1];
% fcorr = zeros(3,length(franka_force));
% for j=1:length(franka_force)
%     fcorr(1:3,j) = R45*franka_force(1:3,j);
% end
