Ac = [0 1 0 0;
    0 0 -52.0624 0;
    0 0 0 1;
    0 0 76.1771 0]

Bc = [0;
    34.4136;
    0;
    -13.3529]

Cc = eye(4)

csys = ss(Ac, Bc , Cc, 0)

dsys = c2d(csys, 0.005, 'zoh')

%zp = pole(dsys)
cp = eig(Ac)
zp = eig(dsys.A)

%cpoles = [-25 -16 -4 -8.71]
cpoles = [-9.5 -10.5 -11 -12]
zpoles = exp(cpoles * 0.005)

K = place(dsys.A, dsys.B, zpoles)

vel_c = tf([100 0], [1 100])
vel_d = c2d(vel_c, 0.005, 'Tustin')