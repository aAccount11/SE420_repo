wn = 9.7087 * 2 * pi;
gain = 0.352500;
zeta = 0.0035;


G_s = tf([gain * wn^2], [1, 2 * zeta * wn, wn^2])
G_z = c2d(G_s, 0.001, "zoh")

diff = tf([400, -400], [1, -0.6], 0.001)
inte = tf([0.001, 0.001], [2, -2], 0.001)

T = sisoinit(4); % The parameter 4 combines an error path controller C1 along with a feedback controller C2
T.G.Value = G_z;
T.OL1.View = {'rlocus'}; % Root Locus plot for error path
T.OL2.View = {'rlocus'} % Root Locus plot for feedback path
controlSystemDesigner(T) % Launch controlSystemDesigner with the above initializations.