data = serialreadSPIRAM("COM6");


time = data(1:400, 1);
vel = data(1:400, 2);
u = data(1:400, 3);
u(1) = 0;
u(2) = 0;
u(3) = 0;

b1 = vel(2:400)
A1 = [vel(1:399) u(1:399)]

b = vel(4: 400);
A = [u(3:399) u(2:398) u(1:397) -vel(3:399) -vel(2:398) -vel(1:397)];

A1\b1
A\b

plot(time, vel, time, u)

%Set up transfer function
num=[60];
den=[.3 1];
sys=tf(num,den)
%Now use c2d to map your transfer function to the z-domain
sysz1=c2d(sys,.01,'zoh')
%Now add a plant numerical error by adding .009 to your discrete model.
sysz1.den{1} = sysz1.den{1} + [0 0.009]
%Now use d2c to map your transfer function back to the s-domain
sysc1=d2c(sysz1)