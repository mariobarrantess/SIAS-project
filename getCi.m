function [C1, C2, C3, C4] = getCi(F)

% Getting Ci(F) full curve
F_plot = [0, 2, 4, 6, 8, 10, 12, 14];

C1_plot = [0, 0.12, 0.22, 0.32, 0.38, 0.47, 0.55, 0.59];
C2_plot = [1.00, 0.79, 0.60, 0.43, 0.30, 0.17, 0.07, 0.01];
C3_plot = [0, 0.10, 0.18, 0.25, 0.32, 0.35, 0.38, 0.40];
C4_plot = [0, 0.17, 0.30, 0.42, 0.50, 0.57, 0.62, 0.66];

F_fine = linspace(0, 14, 200);

% Interpolate the data using 'pchip' to perfectly match points without overshoot
C1_fit = @(x) interp1(F_plot, C1_plot, x, 'pchip', 'extrap');
C2_fit = @(x) interp1(F_plot, C2_plot, x, 'pchip', 'extrap');
C3_fit = @(x) interp1(F_plot, C3_plot, x, 'pchip', 'extrap');
C4_fit = @(x) interp1(F_plot, C4_plot, x, 'pchip', 'extrap');

% % Plot fits vs data
% figure; hold on; grid on;
% plot(F_fine, C1_fit(F_fine), 'k-', 'LineWidth', 1.5);
% plot(F_fine, C2_fit(F_fine), 'k--', 'LineWidth', 1.5);
% plot(F_fine, C3_fit(F_fine), 'k-.', 'LineWidth', 1.5);
% plot(F_fine, C4_fit(F_fine), 'k:', 'LineWidth', 1.5);
% 
% scatter(F_plot, C1_plot, 50, 'filled', 'MarkerFaceColor', 'r');
% scatter(F_plot, C2_plot, 50, 'filled', 'MarkerFaceColor', 'b');
% scatter(F_plot, C3_plot, 50, 'filled', 'MarkerFaceColor', 'g');
% scatter(F_plot, C4_plot, 50, 'filled', 'MarkerFaceColor', 'm');
% 
% legend('C1','C2','C3','C4', 'Location', 'best');
% xlabel('F = (2\pi AR)/(c_{l\alpha} \cdot cos\Lambda_{25})');
% ylabel('C_i Coefficients');
% title('Interpolated Aerodynamic Coefficients (Perfect Match)');

% Selecting Ci for a given F
C1 = C1_fit(F);
C2 = C2_fit(F);
C3 = C3_fit(F);
C4 = C4_fit(F);
end

% function [C1, C2, C3, C4] = getCi(F)
% F_plot = [0, 2, 4, 6, 8, 10, 12, 14];
% C1_plot = [0, 0.12, 0.22, 0.32, 0.38, 0.47, 0.55, 0.59];
% C2_plot = [1.00, 0.79, 0.60, 0.43, 0.30, 0.17, 0.07, 0.01];
% C3_plot = [0, 0.10, 0.18, 0.25, 0.32, 0.35, 0.38, 0.40];
% C4_plot = [0, 0.17, 0.30, 0.42, 0.50, 0.57, 0.62, 0.66];
% 
% F_fine = linspace(0, 14, 200);
% 
% % C1: grows from 0 to ~0.6
% ft1 = fittype('F/(F+a)', 'independent', 'F');
% f1 = fit(F_plot', C1_plot', ft1, 'StartPoint', 10);
% C1 = @(F) F./(F + f1.a);
% 
% % C2: decays from 1 to 0 -> fit as exponential decay
% ft2 = fittype('exp(-a*F)', 'independent', 'F');
% f2 = fit(F_plot', C2_plot', ft2, 'StartPoint', 0.1);
% C2 = @(F) exp(-f2.a * F);
% 
% % C3: bell-shaped
% ft3 = fittype('a*F.*exp(-b*F)', 'independent', 'F');
% f3 = fit(F_plot', C3_plot', ft3, 'StartPoint', [0.1, 0.3]);
% C3 = @(F) f3.a * F .* exp(-f3.b * F);
% 
% % C4: grows like C1 but slower
% ft4 = fittype('F/(F+a)', 'independent', 'F');
% f4 = fit(F_plot', C4_plot', ft4, 'StartPoint', 15);
% C4 = @(F) F./(F + f4.a);
% 
% % Plot fits vs data
% figure; hold on; grid on;
% plot(F_fine, C1(F_fine), 'k-', 'LineWidth', 1.5);
% plot(F_fine, C2(F_fine), 'k--', 'LineWidth', 1.5);
% plot(F_fine, C3(F_fine), 'k-.', 'LineWidth', 1.5);
% plot(F_fine, C4(F_fine), 'k:', 'LineWidth', 1.5);
% 
% scatter(F_plot, C1_plot, 'filled', 'k');
% scatter(F_plot, C2_plot, 'filled', 'k');
% scatter(F_plot, C3_plot, 'filled', 'k');
% scatter(F_plot, C4_plot, 'filled', 'k');
% 
% legend('C_1','C_2','C_3','C_4');
% xlabel('F = (2\pi \cdot AR)/(c_{l\alpha} \cdot \cos\Lambda)');
% ylabel('C_i Coefficients');
% title('Aerodynamic Load Coefficients Fit');
% 
% % Selecting Ci for a given F
% C1 = C1_fit(F);
% C2 = C2_fit(F);
% C3 = C3_fit(F);
% C4 = C4_fit(F);
% end