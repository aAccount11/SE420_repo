% Matthew's work to do the zero order hold thing

wn = 9.7087 * 2 * pi;
gain = 0.352500;
zeta = 0.0035;
T = 0.001;

my_tf = gain * tf( ...
    [wn*wn], ...
    [1 2*zeta*wn wn*wn] ...
);

my_d_tf = c2d(my_tf, T, "zoh");

Kp = 2.5;
Ki = 10.0;
Kd = 0.0625;

initial_zero = -(Ki .* T - 2 .* Kp) / (2.*Kp + Ki.*T);
initial_Kloop = (2.*Kp + Ki.*T) / 2;

T = sisoinit(4);
T.G.Value = my_d_tf;
T.OL1.View = {'rlocus'}; % Root Locus plot for error path
T.OL2.View = {'rlocus'} % Root Locus plot for feedback path
controlSystemDesigner(T) 
% Launch controlSystemDesigner with the above initializations

final_Kloop = 1.6331
final_czero = 0.9525
Kd = 40.0 / 400.0
Kp = (final_Kloop + final_Kloop * final_czero) / 2
Ki = (2*final_Kloop - 2*Kp) / T
