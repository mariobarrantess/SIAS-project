function [trapzwing] = equivalentwing (AC)

% WING GEOMETRY
trapzwing.S_out = 218.9093;
trapzwing.S_fus = 63.6192;
trapzwing.S = 386.8911;
trapzwing.Se = 323.2719;

% Data from Gonzalo's values
% Se = 360; % Exposed surface [m] (from Wikipedia - inventado)
% trapzwing.b = 60.3; % Geometric wingspan (from Airbus webpage) /2 to get each semiwing's wingspan
trapzwing.AR = AC.b^2/AC.S; % Real Wing aspect ratio
trapzwing.ARe = AC.b^2/trapzwing.Se; % Equivalent trapezoid wing aspect ratio

trapzwing.c_bar = trapzwing.Se/AC.b;
trapzwing.c_cle = 10.1; % LE Equivalent wing chord [m]
trapzwing.c_tip = 2.46; % Tip Equivalent wing chord [m]
trapzwing.c_root = 10.1; % Root Equivalent wing chord [m]
trapzwing.taperratio = trapzwing.c_root/trapzwing.c_tip; % [-]

% Angles in the wing planform
trapzwing.lambda_e = trapzwing.c_tip/trapzwing.c_cle; % Equivalent wing taper ratio = c_tip/c_root [deg]
% trapzwing.lambda_e = trapzwing.c_t/trapzwing.c_cle; % Equivalent wing taper ratio [deg] (real)
% trapzwing.lambda_LE = 15; % LE wing taper angle [deg] (inventado)
trapzwing.lambda_LE = 31.47; % LE wing taper angle [deg]
trapzwing.lambda_e_25 = atand(tand(trapzwing.lambda_LE) - 0.5*(trapzwing.c_cle/AC.b)*(1-trapzwing.lambda_e)); % Equivalent wing sweep angle [deg]
trapzwing.lambda_beta = atand(tand(trapzwing.lambda_e_25)/AC.beta); % [deg]

trapzwing.y_target = [0, 8.54, 22.6125];
trapzwing.x_offset = [0, 2.17, 5.71]; % trapezoidal equivalent wing

% c = @(y) trapzwing.c_root*(1-2*y/AC.b*(trapzwing.taperratio-1));

trapzwing.slope = (trapzwing.c_root - trapzwing.c_tip) / (AC.b/2);
c = @(y) trapzwing.c_root - trapzwing.slope * y;

% trapzwing.c.fuselage = c(0);
% trapzwing.c.kink = c(AC.y_kink);
% % c075 = 0.75*AC.b/2;
% trapzwing.c.sec75 = c(0.75*AC.b/2);
% % trapzwing.chords = [trapzwing.c.sec75, trapzwing.c.kink, trapzwing.c.fuselage];
% trapzwing.chords = [trapzwing.c.fuselage, trapzwing.c.kink, trapzwing.c.sec75];

% trapzwing.chords = trapzwing.c_root - ((trapzwing.c_root - trapzwing.c_tip) / AC.b/2) * trapzwing.y_target;

TE_sweep_deg = -20.3; % <--- PLUG IN YOUR EXACT TE ANGLE HERE

% Calculate absolute X-coordinates for the Trailing Edge using trigonometry
trapzwing.TE_X_target = trapzwing.c_root + (trapzwing.y_target .* tand(TE_sweep_deg));

% The local chord is simply the distance between the TE and the LE
trapzwing.chords = trapzwing.TE_X_target - trapzwing.x_offset;

end