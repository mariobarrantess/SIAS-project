%% --- MAIN SIZING SCRIPT: AIRBUS A330-200 WING BOX ---
clear; clc; close all;

% ==========================================
% 1. AIRCRAFT, GEOMETRY & MATERIAL DATA
% ==========================================
[A330, isa] = data_aircraft;

% Equivalent Trapezoidal Wing
trapzwing = equivalentwing (A330);

% Material Properties (AL 2024-T3)
material.rho = 2780;              
material.sigma_y = 324e6;         
material.sigma_allow = 0.70 * material.sigma_y; 
material.tau_allow = material.sigma_allow / sqrt(3); 

% ==========================================
% 2. DESIGN OF EXPERIMENT (DoE) VARIATIONS
% ==========================================
front_spars = [0.15, 0.20, 0.25];
mid_spars   = [0.47, 0.50, 0.53];
rear_spars  = [0.65, 0.70, 0.75];

% ==========================================
% 3. AERODYNAMIC LOADS
% ==========================================
Loads = calcLoadsDiederich(A330, trapzwing, isa);

figure('Name', 'Bending Moment', 'Color', 'w'); 
plot(Loads.y, Loads.Mx/1e6, 'r-', 'LineWidth', 2); grid on;
title('Bending Moment Distribution (Mx)'); 
xlabel('Semi-span [m]'); ylabel('Moment [MN·m]');

% ==========================================
% 4. OPTIMIZATION LOOP (DoE)
% ==========================================
best_weight = inf;
best_config = [0, 0, 0];
best_SecData = cell(3,1);
results = [];

fprintf('Running Wing Box Optimization...\n');


for fs = front_spars
    for ms = mid_spars
        for rs = rear_spars
            % [total_W, SecData] = runFullSizing(fs, ms, rs, A330, Loads, material);
            [total_W, SecData, c] = runFullSizing(fs, ms, rs, A330, trapzwing, Loads, material);
            results = [results; fs, ms, rs, total_W];
            
            if total_W < best_weight
                best_weight = total_W;
                best_config = [fs, ms, rs];
                best_SecData = SecData;
            end
        end
    end
end

% ==========================================
% 5. OUTPUT RESULTS TABLE AND PLOTS
% ==========================================
AllConfigsTable = array2table(results, ...
    'VariableNames', {'Front_Spar', 'Mid_Spar', 'Rear_Spar', 'Total_Mass_kg'});
AllConfigsTable = sortrows(AllConfigsTable, 'Total_Mass_kg');

fprintf('\n======================================================\n');
fprintf('       RESULTS FOR ALL EVALUATED CONFIGURATIONS       \n');
fprintf('======================================================\n');
disp(AllConfigsTable);
fprintf('* Note: Designs with Total_Mass_kg = Inf failed structural limits.\n\n');

fprintf('--- LIGHTEST CONFIGURATION FOUND ---\n');
fprintf('Front Spar: %.2fc | Mid Spar: %.2fc | Rear Spar: %.2fc\n', best_config(1), best_config(2), best_config(3));
fprintf('Total Estimated Wing Box Mass: %.2f kg (Semi-span)\n\n', best_weight);

disp('Sizing Table for ROOT Section (Section 1):');
disp(best_SecData{1}.Table);

fprintf('--- SHEAR CENTER LOCATIONS ---\n');
% for i = 1:3
%     sc_percent = ((best_SecData{i}.X_SC_local - best_SecData{i}.x_booms_local(1)) / A330.chords(i)) * 100;
%     fprintf('Section %d: X_SC = %.2f m global (%.1f%% of local chord behind Front Spar)\n', i, best_SecData{i}.X_SC_global, sc_percent);
% end
num_sections = length(best_SecData);
% chord_local_vect = [c(1), c(29), c(75)]; % approx
for i = 1:num_sections
    % sc_percent = ((best_SecData{i}.X_SC_local - best_SecData{i}.x_booms_local(1)) / trapzwing.chords(i)) * 100;
    sc_percent = ((best_SecData{i}.X_SC_local - best_SecData{i}.x_booms_local(1)) / c(i)) * 100;
    fprintf('Section %d: X_SC = %.2f m global (%.1f%% of local chord behind Front Spar)\n', i, best_SecData{i}.X_SC_global, sc_percent);
end
fprintf('\n');

% ==========================================
% 6. 3D WING BOX AND AIRFOIL PLOT
% ==========================================
figure('Name', '3D Wing Box Extrusion', 'Color', 'w');
hold on; grid on; view(-35, 25); 
title(sprintf('Optimized Wing Box & Airfoils (FS=%.2f, MS=%.2f, RS=%.2f)', ...
    best_config(1), best_config(2), best_config(3)));
xlabel('Chordwise X [m]'); 
ylabel('Spanwise Y [m]'); 
zlabel('Thickness Z [m]');

% 6.1 Plot Airfoils AND Structural Box
% for i = 1:length(A330.y_target)
%     % --- Draw the Outer Aerodynamic Airfoil (Light Grey) ---
%     x_af = best_SecData{i}.X_airfoil;
%     z_af = best_SecData{i}.Z_airfoil;
%     y_af = A330.y_target(i) * ones(size(x_af));
% 
%     plot3(x_af, y_af, z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');
%     plot3(x_af, y_af, -z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');
% 
%     % Close trailing and leading edges of the airfoil
%     plot3([x_af(end), x_af(end)], [y_af(end), y_af(1)], [z_af(end), -z_af(end)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
%     plot3([x_af(1), x_af(1)], [y_af(end), y_af(1)], [z_af(1), -z_af(1)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
% 
%     % --- Draw the Explicit Structural Box (Solid Black) ---
%     x_b = best_SecData{i}.x_booms_local + A330.x_offset(i);
%     z_b = best_SecData{i}.z_upper;
%     y_b = A330.y_target(i) * ones(size(x_b));
% 
%     % Top and Bottom Skins
%     plot3(x_b, y_b, z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3(x_b, y_b, -z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
% 
%     % 3 Vertical Spar Webs
%     plot3([x_b(1), x_b(1)], [y_b(1), y_b(1)], [z_b(1), -z_b(1)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3([x_b(3), x_b(3)], [y_b(3), y_b(3)], [z_b(3), -z_b(3)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3([x_b(5), x_b(5)], [y_b(5), y_b(5)], [z_b(5), -z_b(5)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
% end
% --- NEW ---
% for i = 1:length(A330.y_target)
%     % --- Draw the Outer Aerodynamic Airfoil (Light Grey) ---
%     x_af = best_SecData{i}.X_airfoil;
%     z_af = best_SecData{i}.Z_airfoil;
%     y_af = trapzwing.y_target(i) * ones(size(x_af));
% 
%     plot3(x_af, y_af, z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');
%     plot3(x_af, y_af, -z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');
% 
%     % Close trailing and leading edges of the airfoil
%     plot3([x_af(end), x_af(end)], [y_af(end), y_af(1)], [z_af(end), -z_af(end)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
%     plot3([x_af(1), x_af(1)], [y_af(end), y_af(1)], [z_af(1), -z_af(1)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
% 
%     % --- Draw the Explicit Structural Box (Solid Black) ---
%     x_b = best_SecData{i}.x_booms_local + A330.x_offset(i);
%     z_b = best_SecData{i}.z_upper;
%     y_b = trapzwing.y_target(i) * ones(size(x_b));
% 
%     % Top and Bottom Skins
%     plot3(x_b, y_b, z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3(x_b, y_b, -z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
% 
%     % 3 Vertical Spar Webs
%     plot3([x_b(1), x_b(1)], [y_b(1), y_b(1)], [z_b(1), -z_b(1)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3([x_b(3), x_b(3)], [y_b(3), y_b(3)], [z_b(3), -z_b(3)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
%     plot3([x_b(5), x_b(5)], [y_b(5), y_b(5)], [z_b(5), -z_b(5)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
% end
% 6.1 Plot Airfoils AND Structural Box
for i = 1:length(A330.y_target)
    % --- Draw the Outer Aerodynamic Airfoil (Light Grey) ---
    % Remove the embedded A330 offset and apply the trapzwing offset  
    % so the airfoil aligns with the trapezoidal booms
    x_af_local = best_SecData{i}.X_airfoil - A330.x_offset(i);
    x_af = x_af_local + trapzwing.x_offset(i);

    z_af = best_SecData{i}.Z_airfoil;
    y_af = trapzwing.y_target(i) * ones(size(x_af));

    plot3(x_af, y_af, z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot3(x_af, y_af, -z_af, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % Close trailing and leading edges of the airfoil
    plot3([x_af(end), x_af(end)], [y_af(end), y_af(1)], [z_af(end), -z_af(end)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
    plot3([x_af(1), x_af(1)], [y_af(end), y_af(1)], [z_af(1), -z_af(1)], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);

    % --- Draw the Explicit Structural Box (Solid Black) ---
    % USE trapzwing.x_offset here instead of A330.x_offset
    x_b = best_SecData{i}.x_booms_local + trapzwing.x_offset(i);
    z_b = best_SecData{i}.z_upper;
    y_b = trapzwing.y_target(i) * ones(size(x_b));

    % Top and Bottom Skins
    plot3(x_b, y_b, z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    plot3(x_b, y_b, -z_b, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');

    % 3 Vertical Spar Webs
    plot3([x_b(1), x_b(1)], [y_b(1), y_b(1)], [z_b(1), -z_b(1)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    plot3([x_b(3), x_b(3)], [y_b(3), y_b(3)], [z_b(3), -z_b(3)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    plot3([x_b(5), x_b(5)], [y_b(5), y_b(5)], [z_b(5), -z_b(5)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');

end


% % Add LE & TE lines to plot
% % Leading Edge X coordinates are exactly the x_offset values
% LE_X = trapzwing.x_offset;
% LE_Y = trapzwing.y_target;
% LE_Z = zeros(size(LE_Y)); % Plotting on the Z=0 reference plane
% 
% % Trailing Edge X coordinates are the LE offset + the local chord length
% TE_X = LE_X + trapzwing.chords;
% TE_Y = trapzwing.y_target;
% TE_Z = zeros(size(TE_Y)); % Plotting on the Z=0 reference plane
% 
% % Plot Leading Edge (using a dashed blue line)
% plot3(LE_X, LE_Y, LE_Z, 'b--', 'LineWidth', 2, 'DisplayName', 'Trapezoidal LE');
% 
% % Plot Trailing Edge (using a dashed red line)
% plot3(trapzwing.TE_X_target, TE_Y, TE_Z, 'r--', 'LineWidth', 2, 'DisplayName', 'Trapezoidal TE');



% 6.2 Plot Booms (Stringers and Spar Caps)
% num_booms = length(best_SecData{1}.X);
% for b = 1:num_booms
%     X_line = [best_SecData{1}.X(b), best_SecData{2}.X(b), best_SecData{3}.X(b)];
%     Y_line = [best_SecData{1}.Y(b), best_SecData{2}.Y(b), best_SecData{3}.Y(b)];
%     Z_line = [best_SecData{1}.Z(b), best_SecData{2}.Z(b), best_SecData{3}.Z(b)];
% 
%     plot3(X_line, Y_line, Z_line, '-o', 'LineWidth', 1.5, ...
%         'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'Color', 'b', 'HandleVisibility', 'off');
% end
% 
% % 6.3 Plot Shear Axis
% X_SC_line = [best_SecData{1}.X_SC_global, best_SecData{2}.X_SC_global, best_SecData{3}.X_SC_global];
% Y_line = [best_SecData{1}.Y(1), best_SecData{2}.Y(1), best_SecData{3}.Y(1)];
% Z_line = [0, 0, 0]; 
% 
% plot3(X_SC_line, Y_line, Z_line, '--', 'LineWidth', 3, 'Color', '#32CD32', 'DisplayName', 'Shear Axis');
% legend('show', 'Location', 'best');
% 
% axis equal; 
% set(gca, 'ZDir', 'normal');

% --- NEW ---
% 6.2 Plot Booms (Stringers and Spar Caps)
num_booms = length(best_SecData{1}.X);
for b = 1:num_booms
    X_line = zeros(1, num_sections);
    Y_line = zeros(1, num_sections);
    Z_line = zeros(1, num_sections);

    for s = 1:num_sections
        X_line(s) = best_SecData{s}.X(b);
        Y_line(s) = best_SecData{s}.Y(b);
        Z_line(s) = best_SecData{s}.Z(b);
    end

    plot3(X_line, Y_line, Z_line, '-o', 'LineWidth', 1.5, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'Color', 'b', 'HandleVisibility', 'off');
end

% 6.3 Plot Shear Axis
X_SC_line = zeros(1, num_sections);
Y_line = zeros(1, num_sections);
Z_line = zeros(1, num_sections);

for s = 1:num_sections
    X_SC_line(s) = best_SecData{s}.X_SC_global;
    Y_line(s) = best_SecData{s}.Y(1);
    Z_line(s) = 0; 
end

plot3(X_SC_line, Y_line, Z_line, '--', 'LineWidth', 3, 'Color', '#32CD32', 'DisplayName', 'Shear Axis');
legend('show', 'Location', 'best');
axis equal; 
set(gca, 'ZDir', 'normal');