theta_1_ = deg2rad(77.29);
theta_2_ = deg2rad(38.24);


% Parametros de construccion de la garra
a_ = 0.04;
b_ = 0.02;
gamma_ = 0.9879; % 56.6 degrees -- gamma está en valor absoluto
e_ = 0.028;
d_ = 0.05;
w_ = 0.01; % phalanx width
D_ = 0.08; % distance between parallel fingers
ks1_ = 60;
ks2_ = 675;
ls1r_ = 0.016; 
ls2r_ = 0.057;


p1 = 0.028;%0.03; %28
p2 = 0.012;%0.0098;%12



% Para todas las longitudes posibles de c en una configuracion de agarre
% fija, se calculan para todos los posibles angulos psi_ las fuerzas
% normales a las falanges manteniendo todos los demas parametros ctes. 

c = [0.045:0.001:0.055];
psi_ = [0:0.01:pi];
b_ = [0.005:0.001:0.05];

index_psi = 1;
index_c = 5;
index_b = 16;

Fn1 = zeros(length(c), length(psi_));
Fn2 = zeros(length(c), length(psi_));

for index_c = 1:length(c)
% for index_b = 1:length(b_)
for index_psi = 1:length(psi_)
% Calcular posicion articular del actuador
P1 = [a_*cos(theta_1_); a_*sin(theta_1_)];
P3 = [a_*cos(theta_1_) + b_(index_b)*cos(theta_1_ + theta_2_ - psi_(index_psi)); a_*sin(theta_1_) + b_(index_b)*sin(theta_1_ + theta_2_ - psi_(index_psi))];
Pa = [e_*cos(-gamma_); e_*sin(-gamma_)];
OaO3 = P3-Pa;

theta_a_ = atan2(OaO3(2),OaO3(1)) - acos((d_^2 + c(index_c)^2 - norm(OaO3)^2)/(2*d_*c(index_c)));
% theta_a_ = atan2(OaO3(2),OaO3(1)) - acos((d_^2 + c.^2 - norm(OaO3)^2)./(2*d_*c));

% Calcular angulo muelle
P4 = [e_*cos(-gamma_) + d_*cos(theta_a_); e_*sin(-gamma_) + d_*sin(theta_a_)];
theta_c = atan2(P3(2)-P4(2), P3(1)-P4(1)); % theta_c = theta_4 + theta_d
theta_3_abs = atan2(P1(2)-P3(2), P1(1)-P3(1)); % theta_3_abs = theta_4 + theta_d + theta_3


%% PTV. A partir de aqui se hace el ptv del mecanismo con los parametros dados

%% Jacobiano de los puntos de contacto
% Igual que lo hace Gosselin
r21 = a_*[cos(pi+theta_1_);sin(pi+theta_1_)];
r31 = b_(index_b)*[cos(theta_3_abs);sin(theta_3_abs)] + r21;
r41 = c(index_c)*[cos(theta_c);sin(theta_c)] + r31;
ra1 = d_*[cos(theta_a_);sin(theta_a_)] + r41;

Kind = [     1,       1; ...
       -ra1(2), -r21(2); ...
        ra1(1),  r21(1)];
Kdep = [1,       1,       1; ...
        0, -r41(2), -r31(2); ...
        0,  r41(1),  r31(1)];
C = -inv(Kdep)*Kind;

A = [d_*sin(theta_a_), -b_(index_b)*sin(theta_1_+theta_2_-psi_(index_psi)); -d_*cos(theta_a_), b_(index_b)*cos(theta_1_+theta_2_-psi_(index_psi))];
B = [-a_*sin(theta_1_)-b_(index_b)*sin(theta_1_+theta_2_-psi_(index_psi)), c(index_c)*sin(theta_c); a_*cos(theta_1_)+b_(index_b)*cos(theta_1_+theta_2_-psi_(index_psi)), -c(index_c)*cos(theta_c)];
C2 = inv(B)*A;

C2 = C;

Tn = [sin(theta_1_), cos(theta_1_),  0,                           0; ...
      0,                  0,                   sin(theta_1_+theta_2_), cos(theta_1_+theta_2_)];

Jp1 = [-p1*sin(theta_1_), 0 ; p1*cos(theta_1_),0];
Jp2 = [-a_*sin(theta_1_)-p2*sin(theta_1_+theta_2_), -p2*sin(theta_1_+theta_2_); a_*cos(theta_1_)+p2*cos(theta_1_+theta_2_),  p2*cos(theta_1_+theta_2_)];
Jp = [Jp1;Jp2];

Jpn = Tn * Jp * [C2(1,1), C2(1,2); 0,1];


%% Jacobiano del muelle j
j = norm(P3); theta_j = atan2(P3(2),P3(1));

J2 = [0,a_*b_(index_b)*sin(psi_(index_psi) - theta_2_)/j];

xs1 = a_ * sin(theta_1_ - theta_j); 
tau_2 = ks1_ * (ls1r_ - j) * xs1;
fj = ks1_ * (ls1r_ - j);


%% Jacobiano del actuador
Ja = [1,0]; % El signo - corrige el signo del calculo de las fuerzas por PTV

fc = ks2_ * (ls2r_ - c(index_c)); 
tau_a = fc*d_*cos(theta_c - (theta_a_ + pi/2));


%% Las fuerzas normales en los puntos de contacto por PTV son:
Fpn = -inv(Jpn')* (Ja'*tau_a + J2'*fj); % fc debe ser positiva.


Fn1(index_c, index_psi) = Fpn(1);
Fn2(index_c, index_psi) = Fpn(2);

end
end

[X,Y] = meshgrid(c,psi_);
figure, 
plot3(X,Y,Fn1), grid, % surf(Fn1), 
xlabel('c'), ylabel('psi'), zlabel('fn1');
% zlim([0,10])
figure, 
plot3(X,Y,Fn2), grid, % surf(Fn2), 
xlabel('c'), ylabel('psi'), zlabel('fn2');