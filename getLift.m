function [CL,c_eta] = getLift

% INPUTS
% Se - Wing exposed surface [m^2]
% b - Wingspan [m]
% lambda_e - Equivalent wing taper ratio [deg]
% CL - Aircraft Lift Coefficient
% MTOW - Maximum Take-Off Weight [kg]

% OUTPUTS
% CL = Lift coefficient 2D contour
% c_eta = Equivalent wing chord distribution

% PARAMETERS
% c - Equivalent wing chord distribution
% c_bar - Equivalent wing mean geometric chord
% CL_w - Wing lift coefficient
% L_additional - Additional lift distribution
% epsilon_t - Wing twist angle distribution (tip nose down)
% CL_alpha - Wing lift curve slope
% L_basic - Basic lift distribution

% %% from main
% close all;clear all;clc;  

% data - https://www.aircraft.airbus.com/en/aircraft/a330/a330-200
MTOM = 251000; % Maximum Take-Off Weight [kg]
h_cruise = 12500; % Cruise altitude [m]
M_cruise = 0.82; % Cruise Mach number [-]
beta = sqrt(1-M_cruise^2);

%% WING GEOMETRY
S_out = 218.9093;
S_fus = 63.6192;
S = 386.8911;
Se = 323.2719;

% Data to be updated w/ Gonzalo's values
% Se = 360; % Exposed surface [m] (from Wikipedia - inventado)
b = 60.3; % Geometric wingspan (from Airbus webpage) /2 to get each semiwing's wingspan
AR = b^2/S; % Real Wing aspect ratio
ARe = b^2/Se; % Equivalent trapezoid wing aspect ratio

c_bar = Se/b;
c_cle = 10.1; % LE Equivalent wing chord [m]
c_tip = 2.46; % Tip Equivalent wing chord [m]

% Angles in the wing planform
lambda_e = c_tip/c_cle; % Equivalent wing taper ratio = c_tip/c_root [deg]
% lambda_e = c_t/c_cle; % Equivalent wing taper ratio [deg] (real)
% lambda_LE = 15; % LE wing taper ratio [deg] (inventado)
lambda_LE = 31.47; % LE wing taper ratio [deg]
lambda_e_25 = atand(tand(lambda_LE) - 0.5*(c_cle/b)*(1-lambda_e)); % Equivalent wing sweep angle [deg]
lambda_beta = atand(tand(lambda_e_25)/beta); % [deg]
%% INSIDE THE FUNCTION

g = 9.80665; % Gravitational acceleration [m/s^2]
n_z = 2.5; % Pull-up load factor
rho_0 = 1.225; % Air density at sea level [kg/m3]
a_0 = 340.29; % Speed of sound at sea level [m/s]

MTOW = MTOM*g; % Maximum Take-Off Weight [N]

theta_c = 1-2.25577e-5*h_cruise;
sigma_c = theta_c^4.2561;
rho_c = rho_0 * sigma_c;
V_c = M_cruise*a_0*sqrt(theta_c);
V_c_kmh = V_c*3600/1000;

%%
CL = (2*n_z*MTOW)/(Se*rho_c*V_c^2);

% lambda_beta = atand(lambda_e_25/beta); % Equivalent trapezoid wing compressible sweep angle [deg]
CL_w = 1.5*CL; % Placeholder for wing lift coefficient, update with actual value
cl_alpha = (2*pi)/(sqrt(1-(M_cruise*cosd(lambda_e_25))^2)); % Airfoil lift coefficient slope


CL_alpha = (2*pi*ARe)/(2+sqrt(((2*pi*ARe)/(cosd(lambda_beta)*cl_alpha))^2+4));

%% Wingspan coordinate - eta
y = linspace(0,b/2,100);
eta = zeros(length(b));
for i = 1:length(y)
    eta(i)= 2*y(i)./b;
end

c_eta = c_cle * (1 - (1-lambda_e).*eta);


%% Additional Lift (Diederich's method)
F = (2*pi*AR)/(cl_alpha*cosd(lambda_e_25));

% simply select the value for each Ci constant that corresponds to the value of F i have obtained
[C1, C2, C3, C4] = getCi(F);

% From Gonzalo's Picture of Equivalent Trapezoidal Wing
c_root = 10.1;
c_tip = 2.46;

[c] = getChord (b, c_root, c_tip, eta);

L_additional = zeros(1,length(eta));

for i = 1:length(eta)
    L_additional (i) = C1*(c(i)/c_bar) + (C2+C3)*(4/pi)*sqrt(1-eta(i)^2);
end
% revisar si los resultados tienen sentido

%% Basic Lift distribution (Diederich's method)

L_basic = zeros(1,length(eta));
twist_angle = zeros(1,length(eta));
Cl = zeros(1,length(eta));
Cl_ceta = zeros(1,length(eta));

% How to get alpha_01 (Integral[0-1] epsilon(eta)/epsilon_t * L_additional * d_eta
% dalpha_01 = @ (eta) epsilon/epsilon_t * L_additional;
% alpha_01 = integral(dalpha_01, 0, 1);
%alpha_01=0 since the 3 airfoils composing the wing are all symmetric
% therefore the zero-lift angle of attack is exactly zero
alpha_01 = 0;
epsilon_t = 3; % Wing twist angle [deg] 1º-3º chose the largest since a330-200 is big aircraft

for i = 1:length(eta)
    epsilon_eta(i) = eta(i) * epsilon_t;
    twist_angle (i) = eta (i); % Linear twist angle distribution = epsilon(i)/epsilon_t
    L_basic (i) = L_additional(i).*C4.*(twist_angle (i) - alpha_01);
    
    Cl(i) = c_bar/c_eta(i) *(CL_w.*L_additional(i) + epsilon_t*CL_alpha*L_basic(i));
    Cl_ceta(i) = Cl(i) * c_eta(i);
end

L_total = L_additional + L_basic;

figure
plot(eta, L_total)
hold on
plot(eta, L_basic)
hold on
plot(eta, L_additional)
xlabel('eta')
ylabel('$L_{total}$',Interpreter='latex')
legend('L_{total}', 'L_{basic}', 'L_{additional}', location='best')

% % PLOTTING EACH CONTRIBUTION OF LIFT SEPARATELY
% figure
% plot(eta, L_basic)
% xlabel('eta')
% ylabel('$L_{basic}$',Interpreter='latex')
% 
% figure
% plot(eta, L_additional)
% xlabel('eta')
% ylabel('$L_{additional}$',Interpreter='latex')
% 
% figure
% plot(eta, L_additional)
% xlabel('eta')
% ylabel('$L_{additional}$',Interpreter='latex')

% Cl = c_bar./c_eta *(CL_w.*L_additional + epsilon_t.*CL_alpha.*L_basic);
% 
% Cl_ceta = Cl * c_eta;

figure
plot(eta, Cl_ceta)
hold on
plot(eta, Cl)
title('Wing lift distribution', 'FontSize', 14, 'FontWeight', 'bold' , Interpreter='latex')
xlabel('$eta = \frac{2y}{b}$', Interpreter='latex')
ylabel('${C_l(\eta)}{c(\eta)}$',Interpreter='latex')
legend('{C_l(\eta)}{c(\eta)}','Cl')


end