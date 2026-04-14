function [A330, isa] = data_aircraft

A330.b = 60.3;              
A330.S = 361.6;             

% data - https://www.aircraft.airbus.com/en/aircraft/a330/a330-200
A330.MTOM = 251000;    % Maximum Take-Off Weight [kg]
A330.nz = 2.5;    % Load factor - L/W [-]          
A330.h_cruise = 12500; % Cruise altitude [m]
A330.M_cruise = 0.82;  % Cruise Mach number [-]
A330.beta = sqrt(1-A330.M_cruise^2);

A330.y_kink = 8.54;
A330.y_target = [0, 8.54, 22.6125]; 
A330.chords = [10.1, 7.667, 4.2]; 
A330.airfoils = {'NACA0014.dat', 'NACA0012.dat', 'NACA0009.dat'};

% Sweep Offsets 
A330.x_offset = [0, 3.4, 12.35]; % real wing
% A330.x_offset = [0, 2.17, 5.71]; % trapezoidal equivalent wing

isa.g = 9.80665; % Gravitational acceleration [m/s^2]
isa.rho_0 = 1.225; % Air density at sea level [kg/m3]
isa.a_0 = 340.29; % Speed of sound at sea level [m/s]

A330.MTOW = A330.MTOM*isa.g; % Maximum Take-Off Weight [N]

isa.theta_c = 1-2.25577e-5*A330.h_cruise;
isa.sigma_c = isa.theta_c^4.2561;
isa.rho_c = isa.rho_0 * isa.sigma_c;
A330.V_c = A330.M_cruise*isa.a_0*sqrt(isa.theta_c);
A330.V_c_kmh = A330.V_c*3600/1000;

end