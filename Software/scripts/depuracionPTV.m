% Puntos de contacto
% if ~exist('p1')
    p1 = 0.02;%0.03; %28
% end
% if ~exist('p2')
    p2 = 0.02;%0.0098;%12
% end

% Parametros de construccion de la garra
a_ = 0.04;
b_ = 0.02;
psi_ = pi/2;
gamma_ = 0.9879; % 56.6 degrees -- gamma está en valor absoluto
e_ = 0.028;
d_ = 0.05;
w_ = 0.01; % phalanx width
D_ = 0.08; % distance between parallel fingers
ks1_ = 60;
ks2_ = 675;
ls1r_ = 0.016; 
ls2r_ = 0.057;

lambda_dedo = [1,-1,-1,1];

theta_a = [theta_a_0;theta_a_1;theta_a_2;theta_a_3];
theta_1 = [theta_1_0;theta_1_1;theta_1_2;theta_1_3];
theta_2 = [theta_2_0;theta_2_1;theta_2_2;theta_2_3];

tau_a_ = zeros(size(theta_a));

tau_aJ = zeros(size(theta_a));
Fptvfull = zeros(4,length(theta_a));
Fptvfull = zeros(4,length(theta_a));
dedo = 1;
index = 5541;
for dedo=1:4
Fptv1 = []; Fptvxy = [];

for index=1:length(theta_a)
    
theta_a_ = theta_a(dedo,index);
theta_1_ = theta_1(dedo,index);
theta_2_ = theta_2(dedo,index);

P1 = [a_*cos(theta_1_); a_*sin(theta_1_)];
P2 = [a_*cos(theta_1_) + b_*cos(theta_1_ + theta_2_ - psi_); a_*sin(theta_1_) + b_*sin(theta_1_ + theta_2_ - psi_)];
P3 = [e_*cos(-gamma_) + d_*cos(theta_a_); e_*sin(-gamma_) + d_*sin(theta_a_)];
c = norm(P2-P3);
theta_c = atan2(P2(2)-P3(2), P2(1)-P3(1)); % theta_c = theta_4 + theta_d
theta_3_abs = atan2(P1(2)-P2(2), P1(1)-P2(1)); % theta_3_abs = theta_4 + theta_d + theta_3



%% Jacobiano de los puntos de contacto
% Igual que lo hace Gosselin
r21 = a_*[cos(pi+theta_1_);sin(pi+theta_1_)];
r31 = b_*[cos(theta_3_abs);sin(theta_3_abs)] + r21;
r41 = c*[cos(theta_c);sin(theta_c)] + r31;
ra1 = d_*[cos(theta_a_);sin(theta_a_)] + r41;

Kind = [     1,       1; ...
       -ra1(2), -r21(2); ...
        ra1(1),  r21(1)];
Kdep = [1,       1,       1; ...
        0, -r41(2), -r31(2); ...
        0,  r41(1),  r31(1)];
C = -inv(Kdep)*Kind;


A = [d_*sin(theta_a_), -b_*sin(theta_1_+theta_2_-psi_); -d_*cos(theta_a_), b_*cos(theta_1_+theta_2_-psi_)];
B = [-a_*sin(theta_1_)-b_*sin(theta_1_+theta_2_-psi_), c*sin(theta_c); a_*cos(theta_1_)+b_*cos(theta_1_+theta_2_-psi_), -c*cos(theta_c)];
C2 = inv(B)*A;

C2 = C;

Tn = [sin(theta_1_), cos(theta_1_),  0,                           0; ...
      0,                  0,                   sin(theta_1_+theta_2_), cos(theta_1_+theta_2_)];

Jp1 = [-p1*sin(theta_1_), 0 ; p1*cos(theta_1_),0];
Jp2 = [-a_*sin(theta_1_)-p2*sin(theta_1_+theta_2_), -p2*sin(theta_1_+theta_2_); a_*cos(theta_1_)+p2*cos(theta_1_+theta_2_),  p2*cos(theta_1_+theta_2_)];
Jp = [Jp1;Jp2];

Jpn = Tn * Jp * [C2(1,1), C2(1,2); 0,1];


%% Jacobiano del muelle j
j = norm(P2); theta_j = atan2(P2(2),P2(1));

J2 = [0,a_*b_*cos(theta_2_)/j];

xs1 = a_ * sin(theta_1_ - theta_j); 
tau_2 = ks1_ * (ls1r_ - j) * xs1;
fj = ks1_ * (ls1r_ - j);

tau_2_(dedo,index) = tau_2;

%% Jacobiano del actuador
Ja = [1,0]; % El signo - corrige el signo del calculo de las fuerzas por PTV

fc = ks2_ * (ls2r_ - c); 
tau_a = fc*d_*cos(theta_c - (theta_a_ + pi/2));

tau_a_(dedo,index) = tau_a;


%% Las fuerzas normales en los puntos de contacto por PTV son:
Fpn = -inv(Jpn')* (Ja'*tau_a + J2'*fj); % fc debe ser positiva

Fptv1 = [Fptv1,Fpn];
Fptvfull1(dedo,index) = Fpn(1);
Fptvfull2(dedo,index) = Fpn(2);

%% Calculo de las fuerzas en XY de cada metodo
% Trans = [lambda_dedo(dedo)*-sin(theta_1_), lambda_dedo(dedo)*-sin(theta_1_+theta_2_);cos(theta_1_), -cos(theta_1_+theta_2_)];
Trans = [lambda_dedo(dedo)*cos(theta_1_+pi/2), lambda_dedo(dedo)*cos(pi/2+theta_1_+theta_2_);sin(pi/2+theta_1_), sin(pi/2+theta_1_+theta_2_)];
Fptvxy = [Fptvxy; Trans*[Fpn(1),0;0,Fpn(2)]];


end

if dedo == 1
    Fxd11 = [Fptvxy(1:2:end,1)];
    Fxd12 = [Fptvxy(1:2:end,2)];
    Fyd11 = [Fptvxy(2:2:end,1)];
    Fyd12 = [Fptvxy(2:2:end,2)];
elseif dedo==2
    Fxd21 = [Fptvxy(1:2:end,1)];
    Fxd22 = [Fptvxy(1:2:end,2)];
    Fyd21 = [Fptvxy(2:2:end,1)];
    Fyd22 = [Fptvxy(2:2:end,2)];
elseif dedo==3
    Fxd31 = [Fptvxy(1:2:end,1)];
    Fxd32 = [Fptvxy(1:2:end,2)];
    Fyd31 = [Fptvxy(2:2:end,1)];
    Fyd32 = [Fptvxy(2:2:end,2)];
else
    Fxd41 = [Fptvxy(1:2:end,1)];
    Fxd42 = [Fptvxy(1:2:end,2)];
    Fyd41 = [Fptvxy(2:2:end,1)];
    Fyd42 = [Fptvxy(2:2:end,2)];
end

end

forces_horizontal = ((Fxd11+Fxd41+Fxd12+Fxd42) - (Fxd21+Fxd31+Fxd22+Fxd32))/2;
forces_vertical = ((Fyd11+Fyd41+Fyd21+Fyd31) - (Fyd12+Fyd42+Fyd22+Fyd32))/2;


% figure, plot(forces_horizontal)
% hold on, plot(fsensor)
figure, plot(Fptv1')