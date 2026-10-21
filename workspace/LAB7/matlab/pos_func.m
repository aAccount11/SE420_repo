clc
vel_func = tf([0.1757], [1, -0.9976], 0.001)
vel_func_cont = d2c(vel_func)
step = tf([1], [1, 0]);
derivative_cont = step * vel_func_cont
pos_func_d = c2d(derivative_cont, 0.001)