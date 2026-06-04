function traj = generateDroneTrajectory(cfg)
% generateDroneTrajectory
% Discrete drone trajectory (distance-sampled)
%
% cfg:
%   .type               % 'line' | 'circle' | 'figure8' | 'lawnmower'
%   .centerLat, .centerLon
%   .alt_m
%   .step_m             % distance between points (e.g. 10 m)
%   .totalDist_m
%
% type-specific:
%   line:      azimuth_deg
%   circle:    radius_m
%   figure8:   amplitude_m
%   lawnmower: width_m, length_m, nLines

    metersPerDegLat = 111320;
    metersPerDegLon = @(lat) 111320*cosd(lat);

    % Discrete accumulated distance
    s = (0:cfg.step_m:cfg.totalDist_m).';
    N = numel(s);

    switch lower(cfg.type)

        case 'line'
            az = cfg.azimuth_deg;
            x = s * cosd(az);
            y = s * sind(az);

        case 'circle'
            R = cfg.radius_m;
            theta = s / R;
            x = R * cos(theta);
            y = R * sin(theta);

        case 'figure8'
            A = cfg.amplitude_m;
            theta = s / A;
            x = A * sin(theta);
            y = A * sin(2*theta)/2;

        case 'lawnmower'
            [x,y] = lawnmowerPath(cfg, s);

        otherwise
            error('Unsupported trajectory type');
    end

    % Convert to lat/lon
    traj.lat = cfg.centerLat + y / metersPerDegLat;
    traj.lon = cfg.centerLon + x ./ metersPerDegLon(cfg.centerLat);
    traj.alt_m = cfg.alt_m * ones(1,N);

    traj.N = N;
end
