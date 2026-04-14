function [x_clean, z_clean] = loadAirfoilDat(filename, chord)
    data = load(filename); % Load the 2-column .dat file
    x_raw = data(:,1);
    z_raw = data(:,2);
    
    % 1. Extract only the upper surface (where Z is positive or zero)
    % This prevents the X coordinates from looping back on themselves
    upper_idx = z_raw >= -1e-6; % Using -1e-6 to catch numerical zeros safely
    x_upper = x_raw(upper_idx);
    z_upper = z_raw(upper_idx);
    
    % 2. Remove any remaining duplicate X points (like exactly at the LE/TE)
    % The 'unique' function also automatically sorts X ascendingly, which interp1 needs!
    [x_unique, unique_idx] = unique(x_upper);
    z_unique = z_upper(unique_idx);
    
    % 3. Scale by the local chord length
    x_clean = x_unique * chord;
    z_clean = z_unique * chord;
end