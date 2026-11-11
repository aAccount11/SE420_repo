% This M-file gets you started in doing identification of the Pendubot
% linkage.  This file calculates the parameters of the linkage by making
% some assumptions and then calculating the parameters.  Most of the
% parts of the Pendubot I was able to weigh to get a mass value.
% both Center of Mass and Moments of Inertia are calculated by hand here in
% the script file.  These parameters will work to control the Pendubot.
g = 9.81
Imotorshaft = 9.8839e-7;; %Kg-m^2   Given by the manufacturer 1.4e-4 oz-in-s^2
mmotorshaft = 2*((((.003175)^2)/4)*pi*0.0889*7805.733); % Kg    guess - mass of motor's roter
% just a guess taking the mass of 2 motor shafts as a guess of the motor mass
                  % This is a guess and may be in error.  I am assuming that
                  % most of the mass of the motor is in the coil and since that
                  % is part of the torque generation unit the coil's mass is not
                  % included in the mass of link one.  So here I am considering
                  % the mass added to link one by the motor is just a steel shaft of
                  % length .0889m (3.5in) and diameter .003175m (0.125in).  
mhub1 = 0.0074;  % Kg  mass of link one's mounting hub
ml1p = 0.0104;  % Kg  mass of link one's leg
menc = 0.0163;  %  Kg  mass of the optical encoder minus 4 grams for shaft
Ienc = (menc/12)*(0.03^2+0.013^2);  % Kg-m^2   guess   Moment of Inertia of the Optical Encoder
menc_shaft = 0.004;   % Kg  mass of encoder's shaft (part of link 2) again a slight quess at the encoder's shaft's mass
ml2p = 0.0093;  % Kg  mass of link two's leg
mhub2 = 0.0061;  % Kg  mass of link two's mounting hub
mnuts = 0.0045; % Kg  mass of nuts at the end of link two

Ihub1 = (mhub1/8)*(0.011176^2+ 0.003175^2); % Kg-m^2  I of link one's mounting hub about its center

Il1p = (0.0078/12)*(0.01905^2 + 0.073^2) + (0.0026/12)*(0.01905^2 + 0.00238^2) + 0.0026*(0.073 - 0.0454)^2; % Kg-m^2  I of link one's leg about its centroid

Ienc_shaft = (menc_shaft/8)*0.00635^2;  % Kg-m^2  I of encoder's shaft

Il2p = (ml2p/48)*(3*0.0048006^2 + 4*0.20955^2); % Kg-m^2  I of link two's Leg

Ihub2 = (mhub2/8)*(0.022225^2 + 0.00635^2); % Kg-m^2  I of link two's mounting hub

% Total mass of link one
ml1 = mmotorshaft + mhub1 + ml1p + menc;

% center of mass of link one
lc1 = (ml1p*0.0454 + menc*0.065)/ml1;

% Total moment of Inertia of Link one about its center of mass
Il1 = Ihub1 + mhub1*lc1^2 + Il1p + ml1p*(lc1 - 0.0454)^2;
Il1 = Il1 + Ienc + menc*(0.065-lc1)^2;

% Total mass of link two
ml2 = menc_shaft + ml2p + mhub2 + mnuts;
% Center of mass of link two
lc2 = (ml2p*0.109855 + mnuts*0.20955)/ml2;

% Total moment of Inertia of Link Two about its center of mass
Il2 = Ienc_shaft + menc_shaft*lc2^2 + Il2p + ml2p*(lc2 - 0.109855)^2 + Ihub2 + mhub2*lc2^2;
Il2 = Il2 + mnuts*(0.20955-lc2)^2;

% generate the six parameters
L0 = 0.068;
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
TorqueConst = 0.0049431; %N-m/Vin  (Vin is voltage applied to amplifier)
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

