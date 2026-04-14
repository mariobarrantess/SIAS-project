function [A, t_skin, t_web, W, RF_boom, RF_skin_min, Z, X_SC] = fullyStressedIteration(x_airfoil, z_airfoil, x_booms_local, Mx, Vz, mat)
    % 1. GEOMETRY SETUP (10 Booms, 3 Webs, 2 Cells)
    z_upper = interp1(x_airfoil, z_airfoil, x_booms_local, 'linear', 'extrap');
    z_lower = -z_upper; 
    Z = [z_upper, z_lower]'; 
    num_booms = length(Z); 
    
    dx = diff(x_booms_local);
    dz = diff(z_upper);
    L_skin = sqrt(dx.^2 + dz.^2); 
    L_web = 2 * z_upper([1, 3, 5]); 
    
    dist_booms = sqrt(diff(x_booms_local).^2 + diff(z_upper).^2);
    max_diameter = zeros(1, 5);
    max_diameter(1) = dist_booms(1);           
    max_diameter(end) = dist_booms(end);       
    for b = 2:4
        max_diameter(b) = min(dist_booms(b-1), dist_booms(b)); 
    end
    A_max = [pi * (max_diameter / 2).^2, pi * (max_diameter / 2).^2]'; 
    
    A = 500e-6 * ones(num_booms, 1);
    t_skin = 1e-3 * ones(4, 1);
    t_web  = 1e-3 * ones(3, 1); 
    
    converged = false;
    iter = 0;
    
    % 2. SIZING ITERATION LOOP
    while ~converged && iter < 1000
        iter = iter + 1;
        Ixx = sum(A .* Z.^2);
        
        sigma = (Mx * Z) / Ixx;
        RF_boom = mat.sigma_allow ./ (abs(sigma) + 1e-9); 
        A_old = A;
        A = A ./ RF_boom;
        
        A(A < 100e-6) = 100e-6; 
        A(A > A_max) = A_max(A > A_max);
        
        Vz_calc = Vz;
        if abs(Vz) < 1e-3; Vz_calc = 1; end 
        
        dF = - (Vz_calc / Ixx) * (A(1:5) .* Z(1:5));
        dF_bot = - (Vz_calc / Ixx) * (A(6:10) .* Z(6:10));
        
        qb_top = zeros(4,1); qb_bot = zeros(4,1); qb_web = zeros(3,1);
        
        qb_top(1) = 0; 
        qb_top(2) = qb_top(1) + dF(2); 
        qb_top(3) = 0; 
        qb_top(4) = qb_top(3) + dF(4); 
        
        qb_web(3) = qb_top(4) + dF(5); 
        qb_bot(4) = qb_web(3) + dF_bot(5); 
        qb_bot(3) = qb_bot(4) + dF_bot(4); 
        
        qb_web(2) = qb_top(2) - qb_top(3) + dF(3);
        qb_bot(2) = qb_bot(3) + qb_web(2) + dF_bot(3);
        qb_bot(1) = qb_bot(2) + dF_bot(2); 
        
        qb_web(1) = 0 - qb_top(1) + dF(1);
        
        ds_t_top = L_skin' ./ t_skin;
        ds_t_bot = L_skin' ./ t_skin; 
        ds_t_web = L_web' ./ t_web;
        
        C = zeros(2,2);
        C(1,1) = sum(ds_t_top(1:2)) + sum(ds_t_bot(1:2)) + ds_t_web(1) + ds_t_web(2);
        C(1,2) = -ds_t_web(2);
        C(2,1) = -ds_t_web(2);
        C(2,2) = sum(ds_t_top(3:4)) + sum(ds_t_bot(3:4)) + ds_t_web(2) + ds_t_web(3);
        
        rhs1 = - ( qb_top(1)*ds_t_top(1) + qb_top(2)*ds_t_top(2) + qb_web(2)*ds_t_web(2) + qb_bot(2)*ds_t_bot(2) + qb_bot(1)*ds_t_bot(1) - qb_web(1)*ds_t_web(1) );
        rhs2 = - ( qb_top(3)*ds_t_top(3) + qb_top(4)*ds_t_top(4) + qb_web(3)*ds_t_web(3) + qb_bot(4)*ds_t_bot(4) + qb_bot(3)*ds_t_bot(3) - qb_web(2)*ds_t_web(2) );
        
        qs0 = C \ [rhs1; rhs2];
        
        q_top = qb_top; q_top(1:2) = q_top(1:2) + qs0(1); q_top(3:4) = q_top(3:4) + qs0(2);
        q_bot = qb_bot; q_bot(1:2) = q_bot(1:2) + qs0(1); q_bot(3:4) = q_bot(3:4) + qs0(2);
        q_web = qb_web; q_web(1) = q_web(1) - qs0(1); q_web(2) = q_web(2) + qs0(1) - qs0(2); q_web(3) = q_web(3) + qs0(2);
        
        if abs(Vz) > 1e-3
            tau_max_skin = max(abs(q_top), abs(q_bot)) ./ t_skin;
            RF_skin_top = mat.tau_allow ./ (tau_max_skin + 1e-9);
            RF_web = mat.tau_allow ./ ((abs(q_web) ./ t_web) + 1e-9);
            
            t_skin_old = t_skin; t_web_old = t_web;
            t_skin = t_skin ./ RF_skin_top;
            t_web = t_web ./ RF_web;
            
            t_skin(t_skin < 1e-3) = 1e-3; 
            t_web(t_web < 1e-3) = 1e-3;
        else
            RF_skin_top = inf * ones(4,1);
            t_skin_old = t_skin; t_web_old = t_web;
        end
        
        if max(abs(A - A_old)./A_old) < 1e-4 && max(abs(t_skin - t_skin_old)./t_skin_old) < 1e-4
            converged = true;
        end
    end
    
    RF_skin_min = min(RF_skin_top);
    if abs(Vz) < 1e-3; RF_skin_min = inf; end
    
    % % 3. SHEAR CENTER CALCULATION
    % x_ref = x_booms_local(1);
    % 
    % % One-line equation to prevent copy-paste dot errors
    % T_webs = -q_web(2) * L_web(2) * (x_booms_local(3) - x_ref) - q_web(3) * L_web(3) * (x_booms_local(5) - x_ref);
    % 
    % T_top = 0; T_bot = 0;
    % for k = 1:4
    %     xA = x_booms_local(k); zA = z_upper(k);
    %     xB = x_booms_local(k+1); zB = z_upper(k+1);
    %     T_top = T_top + q_top(k) * ((xA - x_ref)*zB - (xB - x_ref)*zA);
    %     T_bot = T_bot + q_bot(k) * ((xB - x_ref)*(-zA) - (xA - x_ref)*(-zB));
    % end
    % T_total = T_webs + T_top + T_bot;
    % 
    % X_SC = x_ref - (T_total / Vz_calc);
    
    %%% 3. SHEAR CENTER CALCULATION - GEMINI'S VERSION %%%
    x_ref = x_booms_local(1);
    T_total = 0;

    % TOP SKIN: Flow is Left-to-Right (A to B vector matches flow direction)
    for k = 1:4
        xA = x_booms_local(k) - x_ref; zA = z_upper(k);
        xB = x_booms_local(k+1) - x_ref; zB = z_upper(k+1);
        T_total = T_total + q_top(k) * (xA*zB - xB*zA);
    end

    % BOTTOM SKIN: Flow is Right-to-Left, but A to B vector is Left-to-Right.
    % MUST SUBTRACT instead of add to maintain proper CCW/CW sign convention!
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
            % Web 1 flows Top-to-Bottom, A to B vector is Bottom-to-Top (Subtract)
            T_total = T_total - q_web(w) * (xA*zB - xB*zA);
        else
            % Webs 2 & 3 flow Bottom-to-Top, A to B vector is Bottom-to-Top (Add)
            T_total = T_total + q_web(w) * (xA*zB - xB*zA);
        end
    end

    % Calculate Final Shear Center Local Coordinate
    % Note the positive sign: T = Vz * (X_SC - X_ref) --> X_SC = X_ref + T/Vz
    X_SC = x_ref + (T_total / Vz_calc);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 4. TRUE STRUCTURAL WEIGHT CALCULATION [kg/m]
    W = sum(A * mat.rho) + 2 * sum(t_skin .* L_skin' * mat.rho) + sum(t_web .* L_web' * mat.rho); 
end