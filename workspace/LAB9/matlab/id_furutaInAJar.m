% This M-file gets you started in doing identification of the Furuta
% linkage.  This file calculates the parameters of the linkage by making
% some assumptions and then calculating the parameters.  Most of the
% parts of the Furuta I was able to weigh to get a mass value.
% both Center of Mass and Moments of Inertia are calculated by hand here in
% the script file.  These parameters will work to control the Furuta.
g = 9.81


% I am going to guess at the motor inertia {just the motor inertia not the gear shaft} and say it is about the same is the pittman
Imotor = (9.8839e-7); % Pittman motor Kg-m^2   Given by the manufacturer 1.4e-4 oz-in-s^2


mmotorshaft = pi*(((.0023)^2)/4.0)*0.1*8050;
mmotorshaft = mmotorshaft*4;  % 4 times factor quess for motor coils


mhub1 = 0.0054;  % Kg  mass of link one's mounting hub
Ihub1 = (mhub1/8)*(0.006^2); % Kg-m^2  I of link one's mounting hub about its center  Lumpped into diameter of 6mm shaft


ml1p = 0.0432;  % Kg  mass of link one's leg
Il1p = (ml1p/12.0)*((0.114^2) + (0.059^2)); % Kg-m^2  I of link one's leg about its centroid


menc_shaft = pi*((.006^2)/4.0)*.045*8050;

menc = 0.063-menc_shaft;  %  Kg  mass of the magnet encoder circuit boards
Ienc = (menc/48)*(3*0.038^2+3*0.006^2+ 4*0.018^2);  % Kg-m^2   guess   Moment of Inertia of the Optical Encoder



Iencshaft_link1 = (menc_shaft/48.0)*(3.0*(0.006^2) + 4.0*(0.045^2));
Iencshaft_link2 = menc_shaft*(0.006^2)/8.0;


ml2p = 0.0224;  % Kg  mass of link two's leg
Il2p = (ml2p/48)*(3*0.00465^2 + 4*0.2032^2); % Kg-m^2  I of link two's Leg

mhub2 = 0.006;  % Kg  mass of link two's mounting hub
Ihub2 = (mhub2/8)*(0.02235^2 + 0.006^2); % Kg-m^2  I of link two's mounting hub

mnuts = 0.006; % Kg  mass of nuts at the end of link two



% Total mass of link one
ml1 = mmotorshaft + mhub1 + ml1p + menc;

% center of mass of link one
lc1 = (menc*0.081 + ml1p*0.036)/ml1;

% Total moment of Inertia of Link one about its center of mass
Il1 = Ihub1 + mhub1*lc1^2 + Il1p + ml1p*(0.036-lc1)^2 + Ienc + menc*(0.081-lc1)^2 + Iencshaft_link1 + menc_shaft*(0.086-lc1)^2 + Imotor;

% Total mass of link two
ml2 = menc_shaft + ml2p + mhub2 + mnuts;
% Center of mass of link two
lc2 = (ml2p*0.2032/2 + mnuts*0.2032)/ml2;

% Total moment of Inertia of Link Two about its center of mass
Il2 = Iencshaft_link2 + menc_shaft*lc2^2 + Il2p + ml2p*(lc2 - 0.02032/2)^2 + Ihub2 + mhub2*lc2^2;
Il2 = Il2 + mnuts*(0.2032-lc2)^2;

% generate the six parameters
L0 = 0.104;
l1 = lc2;
m1 = ml2;
J0 = Il1 + ml1*lc1^2;
J1 = Il2;
par(1) = J0 + m1*L0^2;
par(2) = m1*l1^2;
par(3) = J1;
par(4) = m1*L0*l1;
par(5) = m1*l1;


% add the Torque Constant to the parameters
TorqueConst = 0.005649; %N-m/PWM unit  0.8 ozf-in/PWM unit
%TorqueConst = 0.03926; %N-m/PWM unit  5.56 ozf-in/PWM unit
par = par/TorqueConst

Mlin = [par(1) par(4);par(4) par(2)+par(3)];
element = inv(Mlin)*[0;par(5)*g];
Aup = [0 1 0 0;0 0 element(1) 0;0 0 0 1;0 0 element(2) 0]
element2 = inv(Mlin)*[1;0];
Bup = [0;element2(1);0;element2(2)]

clear Mlin element;
Mlin = [par(1) -par(4);-par(4) par(2)+par(3)];
element = inv(Mlin)*[0;-par(5)*g];
Adown = [0 1 0 0;0 0 element(1) 0;0 0 0 1;0 0 element(2) 0]
element2 = inv(Mlin)*[1;0];
Bdown = [0;element2(1);0;element2(2)]

