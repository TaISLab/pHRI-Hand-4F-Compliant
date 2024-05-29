function [] = draw_finger(theta_a_, theta_1_, theta_2_)

a_ = 0.04;
b_ = 0.02;
psi_ = pi/2;
gamma_ = 0.9879; % 56.6 degrees
e_ = 0.028;
d_ = 0.05;
% For future 3D representation
% D_ = 0.08; % distance between parallel fingers

% Forma 1
P2x = a_*cos(theta_1_) + b_*cos(theta_1_ + theta_2_ - psi_);
P2y = a_*sin(theta_1_) + b_*sin(theta_1_ + theta_2_ - psi_);
P3x = e_*cos(-gamma_) + d_*cos(theta_a_);
P3y = e_*sin(-gamma_) + d_*sin(theta_a_);
P2 = [P2x;P2y];
P3 = [P3x;P3y];

c = norm(P2-P3);
theta_c = atan2(P2(2)-P3(2), P2(1)-P3(1));
g = norm(P3);
theta_0 = atan2(P3(2),P3(1));
j = norm(P2);
theta_j = atan2(P2(2),P2(1));

g1 = g; theta_01 = theta_0; 
j1 = j; theta_j1 = theta_j;
c1 = c; theta_c1 = theta_c;

% Forma 2
g = sqrt(e_^2 +  d_^2  + 2 * e_ * d_ * cos(theta_a_ + gamma_));
theta_0 = asin(d_./g .* sin(theta_a_ + gamma_)) - gamma_;
j = sqrt( a_^2  +  b_^2  + 2 * a_ * b_ * sin(theta_2_));
theta_j = theta_1_ - asin(b_./j .* cos(theta_2_));
c = sqrt( g.^2  +  j.^2  - 2.*g.*j.*cos(theta_j - theta_0));
theta_c = pi + theta_0 - asin(j./c .* sin(theta_j - theta_0));

g2 = g; theta_02 = theta_0; 
j2 = j; theta_j2 = theta_j;
c2 = c; theta_c2 = theta_c;


% Compute points
O1 = [0,0];
O2 = [a_*cos(theta_1_), a_*sin(theta_1_)];
O3 = [a_*cos(theta_1_) + b_*cos(theta_1_ + theta_2_ - psi_), a_*sin(theta_1_) + b_*sin(theta_1_ + theta_2_ - psi_)];
O4 = [e_*cos(-gamma_) + d_*cos(theta_a_), e_*sin(-gamma_) + d_*sin(theta_a_)];
O5 = [e_*cos(-gamma_),e_*sin(-gamma_)];
O6 = O2+[a_*cos(theta_1_ + theta_2_), a_*sin(theta_1_ + theta_2_)];

Og1 = [g1*cos(theta_01), g1*sin(theta_01)];
Og2 = [g2*cos(theta_02), g2*sin(theta_02)];
Oj1 = [j1*cos(theta_j1), j1*sin(theta_j1)];
Oj2 = [j2*cos(theta_j2), j2*sin(theta_j2)];

% Plot 
figure();

l12 = line([O1(1), O2(1)],[O1(2), O2(2)]);
l23 = line([O2(1), O3(1)],[O2(2), O3(2)]);
l34 = line([O3(1), O4(1)],[O3(2), O4(2)]);
l45 = line([O4(1), O5(1)],[O4(2), O5(2)]);
l15 = line([O1(1), O5(1)],[O1(2), O5(2)]);
l26 = line([O2(1), O6(1)],[O2(2), O6(2)]);
l36 = line([O3(1), O6(1)],[O3(2), O6(2)]);

lg1 = line([O1(1), Og1(1)],[O1(2), Og1(2)],'Color','r')
lg2 = line([O1(1), Og2(1)],[O1(2), Og2(2)],'Color','g')
lj1 = line([O1(1), Oj1(1)],[O1(2), Oj1(2)],'Color','r')
lj2 = line([O1(1), Oj2(1)],[O1(2), Oj2(2)],'Color','g')


% % % % 
l1inf = line([O1(1), 2*O2(1)],[O1(2), 2*O2(2)],'Color','r')
l3inf = O3-O4;
x = 0:0.001:0.08;
hold on
plot(x,l3inf(2)/l3inf(1) * x + (O3(2)-l3inf(2)/l3inf(1)*O3(1)), 'r');

p2 = 0.02;
Op2 = O2+[p2*cos(theta_1_ + theta_2_), p2*sin(theta_1_ + theta_2_)];
plot(x, tan(theta_1_+theta_2_+pi/2)*x + (Op2(2)-Op2(1)*tan(theta_1_+theta_2_+pi/2)),'k')


grid, axis equal,

end
