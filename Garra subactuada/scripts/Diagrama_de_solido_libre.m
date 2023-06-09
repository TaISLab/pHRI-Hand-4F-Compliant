function [Fn_phalanx] = Diagrama_de_solido_libre(tau_2, fc, b_, theta_c, theta_1_, theta_2_, ks1_, ls1r_, j, theta_j, a_, p1, p2)
% Solucion de las fuerzas de contacto por diagrama de solido libre.
% Es necesario ejecutar antes kinetostatic_model_PTV.m

fp2_mod = (tau_2 + fc*b_*cos(theta_c-theta_1_-theta_2_))/p2;
fc_vec = fc*[cos(theta_c); sin(theta_c)];
fj = ks1_ * (ls1r_ - j);
fj_vec = fj*[cos(theta_j); sin(theta_j)];
fp2_vec = fp2_mod*[cos(theta_1_+theta_2_+pi/2); sin(theta_1_+theta_2_+pi/2)];
f21_vec = fc_vec +fj_vec +fp2_vec;
% f21_vec = -f12_vec;
alfa21 = atan2(f21_vec(2),f21_vec(1));

fp1_mod = norm(f21_vec) * sin(alfa21 - theta_1_) * a_ / p1;
% psi = theta_1_ - atan2(f21_vec(2),f21_vec(1));

Fn_phalanx = [fp1_mod,fp2_mod]';

end