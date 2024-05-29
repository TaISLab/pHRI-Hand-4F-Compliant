function [t] = pwm2torque(pwm)
% Computes the torque exerted by the dynamixel xc430 from the input PWM
% signal according to the curves provided by the manufacturer (Only valid
% in static case) 

t = pwm * 1.6/880;%0.7*9.81*35e-3/270;

end