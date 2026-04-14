function [totalWeight, SecData, c] = runFullSizing(fs, ms, rs, AC, wing, Loads, mat)
       % [totalWeight, SecData] = runFullSizing(fs, ms, rs, Loads, material)
    SecData = cell(3,1);
    W_local = zeros(3,1); 
    failedDesign = false; 

    A_all = cell(3,1); Z_all = cell(3,1); P_all = cell(3,1);
    X_local_all = cell(3,1); X_global_all = cell(3,1);
    L_skin_all = cell(3,1); L_web_all = cell(3,1);
    x_raw_all = cell(3,1); z_raw_all = cell(3,1);

    % =========================================================
    % PASS 1: BENDING SIZING
    % =========================================================
    

    % CHORD %
    y = linspace(0,AC.b/2,100);
    eta = zeros(length(AC.b));
    for i = 1:length(y)
        eta(i)= 2*y(i)./AC.b;
    end
    [c] = getChord (AC.b, wing.c_root, wing.c_tip, eta);
    
    chord_local_vect = [c(1), c(29), c(75)]; % approx

    offset_local_vect = [0, 2.17, 5.71]; % approx

    for i = 1:3
        chord_local = chord_local_vect(i); 
        offset_local = offset_local_vect(i);
        
        Mx = Loads.Mx_target(i);
        
        x_booms_local = [fs, (fs+ms)/2, ms, (ms+rs)/2, rs] * chord_local;
        X_local_all{i} = x_booms_local;
        X_global_all{i} = x_booms_local + offset_local; 
        
        [x_raw, z_raw] = loadAirfoilDat(AC.airfoils{i}, chord_local);
        x_raw_all{i} = x_raw; z_raw_all{i} = z_raw;
        
        z_upper = interp1(x_raw, z_raw, x_booms_local, 'linear', 'extrap');
        Z = [z_upper, -z_upper]'; 
        Z_all{i} = Z;
        
        dx = diff(x_booms_local); dz = diff(z_upper);
        L_skin_all{i} = sqrt(dx.^2 + dz.^2); 
        L_web_all{i} = 2 * z_upper([1, 3, 5]); 
        
        dist_booms = sqrt(diff(x_booms_local).^2 + diff(z_upper).^2);
        max_diameter = zeros(1, 5);
        max_diameter(1) = dist_booms(1);           
        max_diameter(end) = dist_booms(end);       
        for b = 2:4
            max_diameter(b) = min(dist_booms(b-1), dist_booms(b)); 
        end
        A_max = [pi * (max_diameter / 2).^2, pi * (max_diameter / 2).^2]'; 
        
        A = 500e-6 * ones(10, 1);
        for iter = 1:1000
            Ixx = sum(A .* Z.^2);
            sigma = (Mx * Z) / Ixx;
            RF_boom = mat.sigma_allow ./ (abs(sigma) + 1e-9); 
            A_old = A;
            A = A ./ RF_boom;
            A(A < 100e-6) = 100e-6; 
            A(A > A_max) = A_max(A > A_max);
            if max(abs(A - A_old)./A_old) < 1e-4; break; end
        end
        A_all{i} = A;
        Ixx_final = sum(A .* Z.^2);
        P_all{i} = (Mx * Z / Ixx_final) .* A;
    end

    % =========================================================
    % PASS 2: TAPER GRADIENTS
    % =========================================================
    dPdy = cell(3,1); dZdY = cell(3,1); dXdY = cell(3,1);
    X_full = cell(3,1);
    for i=1:3; X_full{i} = [X_global_all{i}, X_global_all{i}]'; end

    dy12 = AC.y_target(2) - AC.y_target(1);
    dy23 = AC.y_target(3) - AC.y_target(2);
    dy13 = AC.y_target(3) - AC.y_target(1);

    dPdy{1} = (P_all{2} - P_all{1}) / dy12;  
    dZdY{1} = (Z_all{2} - Z_all{1}) / dy12;
    dXdY{1} = (X_full{2} - X_full{1}) / dy12;

    dPdy{2} = (P_all{3} - P_all{1}) / dy13;  
    dZdY{2} = (Z_all{3} - Z_all{1}) / dy13;
    dXdY{2} = (X_full{3} - X_full{1}) / dy13;

    dPdy{3} = (P_all{3} - P_all{2}) / dy23;  
    dZdY{3} = (Z_all{3} - Z_all{2}) / dy23;
    dXdY{3} = (X_full{3} - X_full{2}) / dy23;

    % =========================================================
    % PASS 3: MULTI-CELL SHEAR SIZING & SHEAR CENTER
    % =========================================================
    for i = 1:3
        dF_top = -dPdy{i}(1:5);
        dF_bot = -dPdy{i}(6:10);
        
        Vz = Loads.Vz_target(i);
        if abs(Vz) < 1e-3; Vz = 1; end 

        t_skin = 1e-3 * ones(4, 1); t_web  = 1e-3 * ones(3, 1);
        L_skin = L_skin_all{i}; L_web = L_web_all{i};

        for iter = 1:1000
            qb_top = zeros(4,1); qb_bot = zeros(4,1); qb_web = zeros(3,1);
            
            % qb_top(1) = 0; qb_top(2) = dF_top(2);
            % qb_top(3) = 0; qb_top(4) = dF_top(4);
            % 
            % qb_web(3) = -qb_top(4) - dF_top(5);
            % qb_bot(4) = qb_web(3) - dF_bot(5);
            % qb_bot(3) = qb_bot(4) - dF_bot(4);
            % 
            % qb_web(2) = qb_top(3) - qb_top(2) - dF_top(3);
            % qb_bot(2) = qb_bot(3) + qb_web(2) - dF_bot(3);
            % qb_bot(1) = qb_bot(2) - dF_bot(2);
            % 
            % qb_web(1) = qb_top(1) - dF_top(1);

            % --- GEMINI'S CORRECTED BASIC SHEAR FLOW EQUILIBRIUM --- %
            qb_top(1) = 0; 
            qb_top(2) = dF_top(2); % Since qb_top(1) is 0
            qb_top(3) = 0; 
            qb_top(4) = dF_top(4); % Since qb_top(3) is 0

            % Web 3 (Rightmost): Flow goes Bottom to Top
            qb_web(3) = -qb_top(4) - dF_top(5);

            % Bottom nodes (Accumulating Right to Left)
            qb_bot(4) = dF_bot(5) - qb_web(3);     % Node 10
            qb_bot(3) = qb_bot(4) + dF_bot(4);     % Node 9

            % Web 2 (Middle): Flow goes Bottom to Top
            qb_web(2) = qb_top(3) - qb_top(2) - dF_top(3);

            % Bottom nodes continued
            qb_bot(2) = qb_bot(3) + dF_bot(3) - qb_web(2); % Node 8
            qb_bot(1) = qb_bot(2) + dF_bot(2);             % Node 7

            % Web 1 (Leftmost): Flow goes Top to Bottom
            qb_web(1) = dF_top(1); % Node 1 -> 6
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            ds_t_top = L_skin' ./ t_skin; ds_t_bot = L_skin' ./ t_skin; ds_t_web = L_web' ./ t_web;
            
            % Twist Matrix (Rigorous CCW integrals)
            C = zeros(2,2);
            C(1,1) = ds_t_bot(1) + ds_t_bot(2) + ds_t_web(2) + ds_t_top(2) + ds_t_top(1) + ds_t_web(1);
            C(1,2) = -ds_t_web(2); 
            C(2,1) = -ds_t_web(2);
            C(2,2) = ds_t_bot(3) + ds_t_bot(4) + ds_t_web(3) + ds_t_top(4) + ds_t_top(3) + ds_t_web(2);
            
            rhs1 = -( qb_bot(1)*ds_t_bot(1) + qb_bot(2)*ds_t_bot(2) + qb_web(2)*ds_t_web(2) ...
                    - qb_top(2)*ds_t_top(2) - qb_top(1)*ds_t_top(1) - qb_web(1)*ds_t_web(1) );
            rhs2 = -( qb_bot(3)*ds_t_bot(3) + qb_bot(4)*ds_t_bot(4) + qb_web(3)*ds_t_web(3) ...
                    - qb_top(4)*ds_t_top(4) - qb_top(3)*ds_t_top(3) - qb_web(2)*ds_t_web(2) );
            
            qs0 = C \ [rhs1; rhs2];
            
            q_bot = qb_bot; q_bot(1:2) = q_bot(1:2) + qs0(1); q_bot(3:4) = q_bot(3:4) + qs0(2);
            q_top = qb_top; q_top(1:2) = q_top(1:2) - qs0(1); q_top(3:4) = q_top(3:4) - qs0(2);
            q_web = qb_web; q_web(1) = q_web(1) - qs0(1); q_web(2) = q_web(2) + qs0(1) - qs0(2); q_web(3) = q_web(3) + qs0(2);
            
            tau_max_skin = max(abs(q_top), abs(q_bot)) ./ t_skin;
            RF_skin = mat.tau_allow ./ (tau_max_skin + 1e-9);
            RF_web = mat.tau_allow ./ ((abs(q_web) ./ t_web) + 1e-9);
            
            t_skin_old = t_skin; 
            t_skin = t_skin ./ RF_skin; t_web = t_web ./ RF_web;
            t_skin(t_skin < 1e-3) = 1e-3; t_web(t_web < 1e-3) = 1e-3;
            
            if max(abs(t_skin - t_skin_old)./t_skin_old) < 1e-4; break; end
        end

        % Node-to-Node Torque Matrix Fix
        A = A_all{i}; Z = Z_all{i}; x_booms_local = X_local_all{i};
        P_Z = P_all{i} .* dZdY{i};
        P_X = P_all{i} .* dXdY{i};

        % x_ref = x_booms_local(1);
        % T_total = 0;
        % 
        % for k = 1:4
        %     xA = x_booms_local(k) - x_ref; zA = z_upper(k);
        %     xB = x_booms_local(k+1) - x_ref; zB = z_upper(k+1);
        %     T_total = T_total + q_top(k) * (xA*zB - xB*zA);
        % end
        % for k = 1:4
        %     xA = x_booms_local(k) - x_ref; zA = -z_upper(k);
        %     xB = x_booms_local(k+1) - x_ref; zB = -z_upper(k+1);
        %     T_total = T_total + q_bot(k) * (xA*zB - xB*zA);
        % end
        % idx_web = [1, 3, 5];
        % for w = 1:3
        %     k = idx_web(w);
        %     xA = x_booms_local(k) - x_ref; zA = -z_upper(k);
        %     xB = x_booms_local(k) - x_ref; zB = z_upper(k);
        %     T_total = T_total + q_web(w) * (xA*zB - xB*zA);
        % end
        % 
        % X_local_full = [x_booms_local, x_booms_local]';
        % for b = 1:10
        %     x = X_local_full(b) - x_ref;
        %     z = Z(b);
        %     T_total = T_total + (x * P_Z(b) - z * P_X(b));
        % end
        
        %%%%% GEMINI'S CORRECTED TORQUE %%%%%%
        x_ref = x_booms_local(1);
        T_total = 0;

        % TOP SKIN: Flow is Left-to-Right (A to B matches flow)
        for k = 1:4
            xA = x_booms_local(k) - x_ref; zA = z_upper(k);
            xB = x_booms_local(k+1) - x_ref; zB = z_upper(k+1);
            T_total = T_total + q_top(k) * (xA*zB - xB*zA);
        end

        % BOTTOM SKIN: Flow is Right-to-Left, but loop is Left-to-Right.
        % MUST SUBTRACT instead of add!
        for k = 1:4
            xA = x_booms_local(k) - x_ref; zA = -z_upper(k);
            xB = x_booms_local(k+1) - x_ref; zB = -z_upper(k+1);
            T_total = T_total - q_bot(k) * (xA*zB - xB*zA);
        end

        % WEBS
        idx_web = [1, 3, 5];
        for w = 1:3
            k = idx_web(w);
            xA = x_booms_local(k) - x_ref; zA = -z_upper(k);
            xB = x_booms_local(k) - x_ref; zB = z_upper(k);

            if w == 1
                % Web 1 flows Top-to-Bottom, loop is Bottom-to-Top (Subtract)
                T_total = T_total - q_web(w) * (xA*zB - xB*zA);
            else
                % Webs 2 & 3 flow Bottom-to-Top, loop is Bottom-to-Top (Add)
                T_total = T_total + q_web(w) * (xA*zB - xB*zA);
            end
        end

        % BOOMS (Taper effects remain unchanged)
        X_local_full = [x_booms_local, x_booms_local]';
        for b = 1:10
            x = X_local_full(b) - x_ref;
            z = Z(b);
            T_total = T_total + (x * P_Z(b) - z * P_X(b));
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % SC properly placed!
        X_SC_local = x_ref + (T_total / Vz); 
        
        SectionLinearMass = sum(A * mat.rho) + 2 * sum(t_skin .* L_skin' * mat.rho) + sum(t_web .* L_web' * mat.rho); 
        W_local(i) = SectionLinearMass;
        
        Ixx = sum(A .* Z.^2);
        sigma = (Loads.Mx_target(i) * Z) / Ixx;
        RF_boom = mat.sigma_allow ./ (abs(sigma) + 1e-9); 
        
        SecData{i}.Table = table((1:10)', A*1e6, RF_boom, 'VariableNames', {'Boom_ID', 'Area_mm2', 'RF_Boom'});
        SecData{i}.SkinTable = table((1:4)', t_skin*1000, 'VariableNames', {'Top_Panel_ID', 'Skin_Thickness_mm'});
        SecData{i}.Linear_Mass_kg_m = SectionLinearMass;
        SecData{i}.X_SC_local = X_SC_local;
        SecData{i}.X_SC_global = X_SC_local + AC.x_offset(i);
        
        X_booms = [X_global_all{i}, X_global_all{i}]'; 
        Y_booms = AC.y_target(i) * ones(size(Z)); 
        
        SecData{i}.X = X_booms; SecData{i}.Y = Y_booms; SecData{i}.Z = Z;
        SecData{i}.X_airfoil = x_raw_all{i} + AC.x_offset(i); SecData{i}.Z_airfoil = z_raw_all{i};
        SecData{i}.x_booms_local = x_booms_local;
        SecData{i}.z_upper = z_upper;
        
        if any(RF_boom < 0.95) || min(RF_skin) < 0.95; failedDesign = true; end
        % if any(RF_boom < 0.999) || min(RF_skin) < 0.999; failedDesign = true; end

    end
    
    if failedDesign
        totalWeight = inf; 
    else
        mass_inner = ((W_local(1) + W_local(2)) / 2) * (AC.y_target(2) - AC.y_target(1));
        mass_outer = ((W_local(2) + W_local(3)) / 2) * (AC.y_target(3) - AC.y_target(2));
        totalWeight = mass_inner + mass_outer; 
    end
end