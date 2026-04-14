function [c] = getChord (b, c_root, c_tip, eta)
% --- Input parameters ---
% c_root - Equivalent root chord [m]
% c_tip - Equivalent tip chord [m]
% b - Total wingspan [m]
% eta = 2y/b - Wingspan coordinate [0, 1] (1x100 vector)

% --- Spanwise coordinate setup ---
% Create an array of y-coordinates from the root (0) to the wingtip (b/2)
num_points = length(eta);
y = linspace(0, b/2, num_points);

% lambda_le = 31.47; % Equivalent Wing Sweep LE Angle [deg]
% lamda_te = 20.30; % Equivalent Wing Sweep TE Angle [deg]
% % c_le = c_root - y*tand(lambda_le);
% c_te = c_root - y*tand(lamda_te);
% % Chord distribution
% c = c_le - c_te;

% --- Calculate Chord Distribution ---
% Linear equation for a trapezoidal wing: c(y) = c_root - slope * y
% The slope is the change in chord divided by the semi-span
slope = (c_root - c_tip) / (b/2);
c = c_root - slope * y;

% % --- Plotting ---
% figure;
% hold on; grid on;
% plot(y, c, 'b-', 'LineWidth', 2);
% 
% % Formatting the plot
% title('Equivalent Trapezoidal Wing: Chord Distribution');
% xlabel('Semi-span location, y [m]');
% ylabel('Chord length, c(y) [m]');
% 
% % Add markers/text for root and tip to make it look professional
% plot(0, c_root, 'ro', 'MarkerFaceColor', 'r');
% plot(b/2, c_tip, 'ro', 'MarkerFaceColor', 'r');
% text(0.5, c_root, sprintf('Root: %.2f m', c_root), 'VerticalAlignment', 'bottom');
% text(b/2 - 0.5, c_tip, sprintf('Tip: %.2f m', c_tip), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% 
% xlim([0, b/2 + 1]);
% ylim([0, c_root + 1]);
end