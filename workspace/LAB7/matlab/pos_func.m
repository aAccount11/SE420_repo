clc
vel_func = tf([0.1757], [1, -0.9976], 0.001)
vel_func_cont = d2c(vel_func)
step = tf([1], [1, 0]);
derivative_cont = step * vel_func_cont
pos_func_d = c2d(derivative_cont, 0.001)
true_d = tf([1,0],[1])
low_pass = tf([500, 0],[1,500])
bode(true_d, low_pass)

tust_prox = c2d(low_pass, 0.001, "tustin")


T = sisoinit(4); % The parameter 4 combines an error path controller C1 along with a feedback controller C2
T.G.Value = pos_func_d
T.OL1.View = {'rlocus'}; % Root Locus plot for error path
T.OL2.View = {'rlocus'} % Root Locus plot for feedback path
%controlSystemDesigner(T) % Launch controlSystemDesigner with the above initializations.