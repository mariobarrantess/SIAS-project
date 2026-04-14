% prueba Loads = calcLoadsDiederich(AC, wing, isa)
    
    clear all; close all; clc;
    
    [AC, isa] = data_aircraft;
    [wing] = equivalentwing (AC);

    
    %%%%%% LIFT %%%%%%%

    CL = (2*AC.nz*AC.MTOW)/(wing.Se*isa.rho_c*AC.V_c^2);

    % wing.lambda_beta = atand(wing.lambda_e_25/AC.beta); % Equivalent trapezoid wing compressible sweep angle [deg]
    CL_w = 1.5*CL; % Placeholder for wing lift coefficient, update with actual value
    cl_alpha = (2*pi)/(sqrt(1-(AC.M_cruise*cosd(wing.lambda_e_25))^2)); % Airfoil lift coefficient slope
    
    
    CL_alpha = (2*pi*wing.ARe)/(2+sqrt(((2*pi*wing.ARe)/(cosd(wing.lambda_beta)*cl_alpha))^2+4));
    
    %% Wingspan coordinate - eta
    y = linspace(0,AC.b/2,100);
    eta = zeros(length(AC.b));
    for i = 1:length(y)
        eta(i)= 2*y(i)./AC.b;
    end
    
    c_eta = wing.c_cle * (1 - (1-wing.lambda_e).*eta);
    
    %% Additional Lift (Diederich's method)
    F = (2*pi*wing.AR)/(cl_alpha*cosd(wing.lambda_e_25));
    
    % simply select the value for each Ci constant that corresponds to the value of F i have obtained
    [C1, C2, C3, C4] = getCi(F);
      
    [c] = getChord (AC.b, wing.c_root, wing.c_tip, eta);
    
    L_additional = zeros(1,length(eta));
    
    for i = 1:length(eta)
        L_additional (i) = C1*(c(i)/wing.c_bar) + (C2+C3)*(4/pi)*sqrt(1-eta(i)^2);
    end   
    %% Basic Lift distribution (Diederich'S method)
    
    L_basic = zeros(1,length(eta));
    twist_angle = zeros(1,length(eta));
    Cl = zeros(1,length(eta));
    Cl_ceta = zeros(1,length(eta));
    
    % How to get alpha_01? - (Integral[0-1] epsilon(eta)/epsilon_t * L_additional * d_eta
    % dalpha_01 = @ (eta) epsilon/epsilon_t * L_additional;
    % alpha_01 = integral(dalpha_01, 0, 1);
    %alpha_01=0 since the 3 airfoils composing the wing are all symmetric
    % therefore the zero-lift angle of attack is exactly zero
    alpha_01 = 0;
    epsilon_t = 3; % Wing twist angle [deg] 1º-3º chose the largest since AC-200 is big aircraft
    
    for i = 1:length(eta)
        epsilon_eta(i) = eta(i) * epsilon_t;
        twist_angle (i) = eta (i); % Linear twist angle distribution = epsilon(i)/epsilon_t
        L_basic (i) = L_additional(i).*C4.*(twist_angle(i) - alpha_01);
        
        Cl(i) = wing.c_bar/c_eta(i) *(CL_w.*L_additional(i) + epsilon_t*CL_alpha*L_basic(i));
        Cl_ceta(i) = Cl(i) * c_eta(i);
    end
    
    % L_total = L_additional + L_basic; % WRONG simplification
    
    % figure
    % plot(eta, L_total)
    % hold on
    % plot(eta, L_basic)
    % hold on
    % plot(eta, L_additional)
    % xlabel('eta')
    % ylabel('$L_{total}$',Interpreter='latex')
    % legend('L_{total}', 'L_{basic}', 'L_{additional}', location='best')
    
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
    
    % Cl = wing.c_bar./c_eta *(CL_w.*L_additional + epsilon_t.*CL_alpha.*L_basic);
    % 
    % Cl_ceta = Cl * c_eta;
    
    % figure
    % plot(eta, Cl_ceta)
    % hold on
    % plot(eta, Cl)
    % title('Wing lift distribution', 'FontSize', 14, 'FontWeight', 'bold' , Interpreter='latex')
    % xlabel('$eta = \frac{2y}{AC.b}$', Interpreter='latex')
    % ylabel('${C_l(\eta)}{c(\eta)}$',Interpreter='latex')
    % legend('{C_l(\eta)}{c(\eta)}','Cl')

    % lift_dist = 1/2* wing.Se * isa.rho_c * AC.V_c^2 * Cl.*c(i);
    q = 0.5 * isa.rho_c * AC.V_c^2; % dynamic pressure
    lift_dist = q * wing.c_bar * (CL * L_additional + (deg2rad(epsilon_t) * CL_alpha * L_basic));

    figure
    plot(eta, lift_dist)
    plot(eta, Cl)
    title('Wing lift distribution', 'FontSize', 14, 'FontWeight', 'bold' , Interpreter='latex')
    xlabel('$eta = \frac{2y}{b}$', Interpreter='latex')
    ylabel('${L(\eta)}$',Interpreter='latex')

    % Loads = restofloads (lift_dist, y, AC);

    %%%%%%%%%%%%%%%%%%%% REST OF LOADS %%%%%%%%%%%%%%%%%%
    
    % Initialize arrays
    Vz = zeros(size(y)); 
    Mx = zeros(size(y));
    
    % 1. Integrate Lift for Shear Force (Vz)
    % We loop up to length(y)-1. The tip shear force remains 0.
    for i = 1:(length(y) - 1)
        Vz(i) = trapz(y(i:end), lift_dist(i:end));
    end
    Vz(end) = 0; % Shear force at the very tip is zero
    
    % 2. Integrate Shear Force for Bending Moment (Mx)
    % We do this in a SECOND loop now that all Vz values are known.
    for i = 1:(length(y) - 1)
        Mx(i) = trapz(y(i:end), Vz(i:end));
    end
    Mx(end) = 0; % Bending moment at the very tip is zero
    
    % Extract values at the target sections using interpolation
    Loads.Mx_target = interp1(y, Mx, AC.y_target);
    Loads.Vz_target = interp1(y, Vz, AC.y_target);
    Loads.y = y; 
    Loads.Mx = Mx;
    Loads.Vz = Vz;