function users = generateUsersInArea(centerLat, centerLon, latSpanDeg, lonSpanDeg, Nusers, groundAlt_m)
% Generates N users randomly distributed within a geographic area
%
% INPUTS:
%   centerLat   - center latitude (degrees)
%   centerLon   - center longitude (degrees)
%   latSpanDeg  - total latitude span (degrees)
%   lonSpanDeg  - total longitude span (degrees)
%   Nusers      - number of users
%   groundAlt_m - ground altitude (m)
%
% OUTPUT:
%   users struct with fields:
%     .N
%     .lat
%     .lon
%     .alt_m

    % Area boundaries
    latMin = centerLat - latSpanDeg/2;
    latMax = centerLat + latSpanDeg/2;
    lonMin = centerLon - lonSpanDeg/2;
    lonMax = centerLon + lonSpanDeg/2;

    % Uniform random distribution
    users.lat = latMin + (latMax - latMin) * rand(Nusers,1);
    users.lon = lonMin + (lonMax - lonMin) * rand(Nusers,1);
    users.alt_m = groundAlt_m * ones(1,Nusers);

    users.N = Nusers;
end
