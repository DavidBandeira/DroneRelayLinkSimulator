clc; clear; close all;

%% Drone configuration
% Definition of the drone trajectory and flight parameters

% Reference geographic position (trajectory origin)
gDT.centerLat   = 39.736052;    % initial latitude (degrees)
gDT.centerLon   = -8.819245;    % initial longitude (degrees)

% Flight parameters
gDT.alt_m       = 1000;         % constant drone altitude (m AGL)
gDT.step_m      = 150;          % distance step between trajectory points (m)
gDT.totalDist_m = 5000;         % total traveled distance along the trajectory (m)

% Trajectory geometry definition
gDT.type = 'line';              % trajectory type: straight line
gDT.azimuth_deg = 90;           % heading angle of the trajectory (degrees)

% Generate the discrete drone trajectory
trajLine = generateDroneTrajectory(gDT);

% Convert trajectory output into a transmitter position structure
dronPos = struct( ...
    "lat", trajLine.lat, ...
    "lon", trajLine.lon, ...
    "alt", trajLine.alt_m);


%% Antenna configuration
% Fixed ground antenna acting as backhaul base station

antPos = struct( ...
    "lat", 39.735052, ...        % antenna latitude
    "lon", -8.819245, ...        % antenna longitude
    "alt", 50);                  % antenna height above ground (m)


%% User configuration
% Definition of the user distribution area and user parameters

centerLat   = 39.74;            % center latitude of user area
centerLon   = -8.82;            % center longitude of user area
latSpanDeg  = 0.001;            % latitude span of the area (degrees)
lonSpanDeg  = 0.001;            % longitude span of the area (degrees)
Nusers      = 4;                % number of ground users
groundAlt_m = 1.5;              % user antenna height above ground (m)

% Generate random user positions within the defined area
users = generateUsersInArea(centerLat, centerLon, ...
                            latSpanDeg, lonSpanDeg, ...
                            Nusers, groundAlt_m);

% Convert users into receiver position structure
userPos = struct( ...
    "lat", users.lat, ...
    "lon", users.lon, ...
    "alt", users.alt_m);


%% Link configuration
% Definition of radio parameters for backhaul and access links


% 5G NR configuration
Cfg_5G = struct( ...
    "fHz", 3.9e9, ...                    % carrier frequency (Hz)
    "BHz", 20e6, ...                     % channel bandwidth (Hz)
    "txPower_dBm", 37, ...               % transmit power (dBm)
    "NF_dB", 7, ...                      % receiver noise figure (dB)
    "rxSens_dBm", -95, ...               % receiver sensitivity (dBm)
    "fadeMargin_dB", 10, ...             % fade margin (dB)
    "propModel", "longley-rice");        % propagation model


% Wi-Fi configuration

Cfg_Wifi = struct( ...
    "fHz", 5e9, ...
    "BHz", 20e6, ...
    "txPower_dBm", 30, ...
    "NF_dB", 7, ...
    "rxSens_dBm", -82, ...
    "fadeMargin_dB", 10, ...
    "propModel", "longley-rice");


%% Map configuration
% Controls visualization of transmitter, receivers and data-rate maps

% Show transmitters, receivers, and throughput contour map
mapCfg1 = struct("Resolution", 20, "MapRange", 2000, ...
                 "tx", true, "rx", true, "Tax", true);

% Show only transmitter and receiver positions
mapCfg2 = struct("Resolution", 20, "MapRange", 2000, ...
                 "tx", true, "rx", true, "Tax", false);

% Disable all visualization
mapCfg3 = struct("Resolution", 20, "MapRange", 2000, ...
                 "tx", false, "rx", false, "Tax", false);


%% Backhaul and local 5G links
% Backhaul: fixed antenna -> UAV
Backhaul_link = computeLink(antPos, dronPos, Cfg_5G, mapCfg2);

% Local access: UAV -> ground users
Local_link = computeLink(dronPos, userPos, Cfg_Wifi, mapCfg2);


%% Results 1 — Link-level metrics
% Time index corresponding to trajectory points
Ntime = numel(dronPos.lat);
t = 0:Ntime-1;


% Backhaul results
figure;
plot(t, Backhaul_link.capacity_Mbps);
xlabel('Positions'); ylabel('Data rate limit (Mbps)');
title('Shannon Limit vs Positions (Backhaul 5G NR)');

figure;
plot(t, Backhaul_link.SNR_dB);
xlabel('Positions'); ylabel('SNR (dB)');
title('5G NR - Backhaul SNR');

figure;
plot(t, Backhaul_link.Prx_dBm);
xlabel('Positions'); ylabel('Prx (dBm)');
title('5G NR - Backhaul received power');


% Local link (mean over users)
Local_meanPrx      = mean(Local_link.Prx_dBm, 2);
Local_meanSNR      = mean(Local_link.SNR_dB, 2);
Local_meanCapacity = mean(Local_link.capacity_Mbps, 2);

figure;
plot(t, Local_meanPrx);
xlabel('Positions'); ylabel('Prx (dBm)');
title('Wi-Fi 6 - Local received power (mean)');

figure;
plot(t, Local_meanSNR);
xlabel('Positions'); ylabel('SNR (dB)');
title('Wi-Fi 6 - Local SNR (mean)');

figure;
plot(t, Local_meanCapacity);
xlabel('Positions'); ylabel('Maximum data rate (Mbps)');
title('Wi-Fi 6 - Local Shannon Limit (mean)');


%% Link availability analysis
% Check if all local user links meet the link budget
allLocalOK = all(Local_link.linkOK, 2);

% Backhaul link availability
backhaulOK = Backhaul_link.linkOK';

% End-to-end link availability (backhaul + local)
totalLinkOK = allLocalOK & backhaulOK;

% Create summary table
LinkStatus = table((1:size(Local_link.linkOK,1))', ...
                   allLocalOK, backhaulOK, totalLinkOK, ...
    'VariableNames', {'TimeStep', 'AllLocalOK', 'BackhaulOK', 'TotalLinkOK'});

disp(LinkStatus)


%% Theoretical estimation of effective throughput
% Mapping SNR to realistic throughput using abstraction models

Backhaul_Mbps = snr2throughput5G(Backhaul_link.SNR_dB);
Local_Mbps    = snr2throughputWiFi(Local_meanSNR);


%% Results 2 — Effective throughput
figure;
plot(t, Backhaul_Mbps, 'o-', 'LineWidth', 1.8);
grid on;
xlabel('Positions'); ylabel('Effective throughput (Mbps)');
title('5G NR – Backhaul Throughput');

figure;
plot(t, Local_Mbps, 'o-', 'LineWidth', 1.8);
grid on;
xlabel('Positions'); ylabel('Effective throughput (Mbps)');
title('Wi-Fi 6 – Local Throughput');



%% Backhaul — Shannon vs Effective Throughput
figure;
hold on; grid on;

% Plot Shannon capacity
plot(t, Backhaul_link.capacity_Mbps, 'LineWidth', 1.8, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Shannon Limit');

% Plot Effective throughput
plot(t, Backhaul_Mbps, 'o-', 'LineWidth', 1.8, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'Effective Throughput');

xlabel('Positions', 'FontSize', 14);
ylabel('Data rate (Mbps)', 'FontSize', 14);
title('5G NR — Shannon Limit vs Effective Throughput (Backhaul)', 'FontSize', 16);
legend('Location', 'best', 'FontSize', 12);
set(gca,'FontSize',12);

hold off;

%% Local link — Shannon vs Effective Throughput
figure;
hold on; grid on;

% Plot Shannon capacity (mean over users)
plot(t, Local_meanCapacity, 'LineWidth', 1.8, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Shannon Limit');

% Plot Effective throughput
plot(t, Local_Mbps, 'o-', 'LineWidth', 1.8, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'Effective Throughput');

xlabel('Positions', 'FontSize', 14);
ylabel('Data rate (Mbps)', 'FontSize', 14);
title('Wi-Fi 6 (5G) — Shannon Limit vs Effective Throughput (Local)', 'FontSize', 16);
legend('Location', 'best', 'FontSize', 12);
set(gca,'FontSize',12);

hold off;