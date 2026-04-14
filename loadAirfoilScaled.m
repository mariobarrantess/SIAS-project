function [x_scaled, z_scaled] = loadAirfoilScaled(filename, chord)
    % Load NACA .dat airfoil coordinates
    coords = load(filename); % assumes [x z]
    
    x = coords(:,1);
    z = coords(:,2);
    
    % Scale with chord
    x_scaled = x * chord;
    z_scaled = z * chord;
end