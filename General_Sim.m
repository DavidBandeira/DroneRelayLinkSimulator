%% UAV Relay Link Simulator  –  v8

clc; clear; close all;


%%  MODO DE TERRENO 


TERRAIN_MODE = 'siteviewer';          % 'tif'  |  'siteviewer'


%%  CONFIGURAÇÃO GERAL


sim.accessTech = '5g';         % '5g'  |  'wifi6'
sim.propModel  = 'longley-rice';

if strcmp(sim.accessTech, 'wifi6')
    cfg.fHz = 5.8e9;
else
    cfg.fHz = 3.8e9;
end

cfg.BHz           = 20e6;
cfg.txPower_dBm   = 23;
cfg.NF_dB         = 7;
cfg.rxSens_dBm    = -100;
cfg.fadeMargin_dB = 10;
cfg.rxGain_dBi    = 0;

cfg.g5.SCS_Hz   = 30e3;
cfg.g5.n_layers = 1;
cfg.g5.overhead = 0.14;

cfg.wifi.nSS   = 1;
cfg.wifi.GI_us = 0.8;

cfg.accessTech = sim.accessTech;
cfg.propModel  = sim.propModel;
cfg.NF_dB_val  = cfg.NF_dB;

backhaul.dl_Mbps   = 100;
backhaul.ul_Mbps   = 15;
backhaul.available = true;

Nusers = 4;

map_cfg.MapRange   = 5000;
map_cfg.Resolution = 150;
map_cfg.type       = 'snr';
nMaps              = 1;


%%  TRAJECTÓRIA BASE  (centro, raio, passo)


gDT.type        = 'circle';         % também é possível 'line'
gDT.radius_m    = 1750;
gDT.step_m      = 50;
gDT.totalDist_m = 2*pi*1750;


%%  BLOCO DE TERRENO 


switch lower(TERRAIN_MODE)

    
    case 'tif'
   

        terrain_name = 'pedrogao_terrain';
        terrain_tif  = 'pedrogao_terrain_wgs84.tif';
        terrain_mat  = 'pedrogao_terrain_light.mat';
        terrain_dt1  = 'pedrogao_terrain_4.dt1';

        % Centro da trajectória
        gDT.centerLat = 39.950822;
        gDT.centerLon = -8.241411;

        % Registar terreno DTED
        try; removeCustomTerrain(terrain_name); catch; end
        if ~isfile(terrain_dt1)
            fprintf('A converter %s → %s ...\n', terrain_tif, terrain_dt1);
            convert_tif_to_dted(terrain_tif, terrain_dt1);
            fprintf('Conversão concluída.\n\n');
        end
        addCustomTerrain(terrain_name, terrain_dt1);
        fprintf('Terreno registado: "%s"\n\n', terrain_name);

        % Carregar renderer
        if isfile(terrain_mat)
            tmp = load(terrain_mat, 'X_light', 'Y_light', 'Z_light');
            terrain_renderer.X = tmp.X_light;
            terrain_renderer.Y = tmp.Y_light;
            terrain_renderer.Z = tmp.Z_light;
            fprintf('Renderer carregado: %s  [%d×%d]\n\n', ...
                    terrain_mat, size(tmp.Z_light,1), size(tmp.Z_light,2));
        else
            warning('"%s" não encontrado – renderer usará plano plano.', terrain_mat);
            terrain_renderer = [];
        end

        % Altitude AGL sobre o pico do terreno (anti-clipping)
        AGL_desired = 1000;
        if ~isempty(terrain_renderer)
            mpdLat = 111320;
            mpdLon = 111320 * cosd(gDT.centerLat);
            lat_t = terrain_renderer.Y(:);
            lon_t = terrain_renderer.X(:);
            z_t   = terrain_renderer.Z(:);
            valid = isfinite(lat_t) & isfinite(lon_t) & isfinite(z_t);
            lat_t = lat_t(valid); lon_t = lon_t(valid); z_t = z_t(valid);
            dx_t = (lon_t - gDT.centerLon) * mpdLon;
            dy_t = (lat_t - gDT.centerLat) * mpdLat;
            in_circle = sqrt(dx_t.^2 + dy_t.^2) <= gDT.radius_m * 1.5;
            fprintf('  Pontos de terreno dentro do raio: %d / %d\n', ...
                    sum(in_circle), numel(z_t));
            if sum(in_circle) > 0
                z_max = max(z_t(in_circle));
                z_mean = mean(z_t(in_circle));
            else
                warning('Nenhum ponto no raio — usando máximo global.');
                z_max  = max(z_t);
                z_mean = mean(z_t);
            end
            gDT.alt_m = z_max + AGL_desired;
            fprintf('  Terreno: max=%.0fm  mean=%.0fm\n', z_max, z_mean);
            fprintf('  Altitude drone: %.0fm MSL  (%.0fm AGL sobre pico)\n\n', ...
                    gDT.alt_m, AGL_desired);
        else
            z_max_fallback = 1000;
            gDT.alt_m = z_max_fallback + AGL_desired;
            fprintf('Sem renderer – altitude drone: %.0fm MSL  (fallback)\n\n', gDT.alt_m);
        end

        % Garantia final
        if isempty(gDT.alt_m) || ~isfinite(gDT.alt_m)
            gDT.alt_m = 1000 + AGL_desired;
            warning('alt_m inválido — fallback para %.0fm', gDT.alt_m);
        end

    
    case 'siteviewer'
    

        % Centro da trajectória 
        gDT.centerLat = 40.4017;
        gDT.centerLon = -7.5433;
        gDT.alt_m     = 2000;      % altitude fixa MSL [m]

        terrain_renderer = [];     % sem renderer
        fprintf('Modo siteviewer – altitude fixa: %.0fm MSL\n\n', gDT.alt_m);

    otherwise
        error('TERRAIN_MODE inválido: "%s"  (use ''tif'' ou ''siteviewer'')', TERRAIN_MODE);
end


%%  ANTENAS


elem_common = struct( ...
    'type','preset','preset','array','fHz',cfg.fHz, ...
    'G_elem_dBi',8,'HPBW_half_deg',30, ...
    'Nx',2,'Ny',2,'eta_element',0.8,'eta_array',0.5);

arrays_cfg(1) = merge_struct(elem_common, struct('Nx',2,'Ny',2,'boresight_az', 90,'boresight_el',-75));
arrays_cfg(2) = merge_struct(elem_common, struct('Nx',2,'Ny',2,'boresight_az',270,'boresight_el',-75));
arrays_cfg(3) = merge_struct(elem_common, struct('Nx',4,'Ny',4,'boresight_az',180,'boresight_el',-20));
arrays_cfg(4) = merge_struct(elem_common, struct('Nx',4,'Ny',4,'boresight_az',160,'boresight_el',-24));
arrays_cfg(5) = merge_struct(elem_common, struct('Nx',4,'Ny',4,'boresight_az',200,'boresight_el',-24));

% Outros exemplos:
%
% Omni:
% arrays_cfg = struct('type','preset','preset','omni','G_elem_dBi',2, ...
%                     'boresight_az',0,'boresight_el',0);
%
% Patch nadir + dipole lateral:
% arrays_cfg(1) = struct('type','preset','preset','patch','G_elem_dBi',7, ...
%                        'boresight_az',0,'boresight_el',-90);
% arrays_cfg(2) = struct('type','preset','preset','dipole','G_elem_dBi',2.15, ...
%                        'boresight_az',0,'boresight_el',0);
%
% LUT .csv:
% arrays_cfg = struct('type','lut','lut_file','antenna_pattern.csv', ...
%                     'boresight_az',0,'boresight_el',-90);



%%  TRAJETÓRIA E ATITUDE


traj  = generateDroneTrajectory(gDT);
Ntime = traj.N;
V_ms  = 165/3.6;
t_vec = (0:Ntime-1) * (gDT.step_m / V_ms);

bank_deg  = rad2deg(atan(V_ms^2 / (gDT.radius_m * 9.81)));
pitch_deg = 3.0;
theta_vec = (0:Ntime-1) * (gDT.step_m / gDT.radius_m);
yaw_vec   = mod(rad2deg(atan2(cos(theta_vec), -sin(theta_vec))), 360);

attitude_vec(Ntime) = struct('roll_deg',0,'pitch_deg',0,'yaw_deg',0);
for k = 1:Ntime
    attitude_vec(k) = struct('roll_deg',-bank_deg, ...
                             'pitch_deg',pitch_deg, ...
                             'yaw_deg',  yaw_vec(k));
end


%%  UTILIZADORES


users = generateUsersInArea(gDT.centerLat, gDT.centerLon, 0.02, 0.02, Nusers, 1.5);
rxPos_array(Nusers) = struct('lat',0,'lon',0,'alt',0);
for u = 1:Nusers
    rxPos_array(u) = struct('lat',users.lat(u),'lon',users.lon(u),'alt',1.5);
end


%%  LOOP TEMPORAL


SNR_mat    = zeros(Ntime, Nusers);
T_mat      = zeros(Ntime, Nusers);
C_mat      = zeros(Ntime, Nusers);
margin_mat = zeros(Ntime, Nusers);
txGain_mat = zeros(Ntime, Nusers);
linkOK_mat = false(Ntime, Nusers);
dist_mat = zeros(Ntime, Nusers);

map_steps = unique(round(linspace(1, Ntime, nMaps)));

simData = struct('txPos',{},'rxPos_array',{},'arrays_cfg',{}, ...
                 'attitude',{},'link',{},'t',{});

fprintf('Tech: %s | Model: %s | Terrain: %s | %d steps | %d users\n', ...
    sim.accessTech, sim.propModel, upper(TERRAIN_MODE), Ntime, Nusers);

for k = 1:Ntime
    txPos  = struct('lat',traj.lat(k),'lon',traj.lon(k),'alt',traj.alt_m(k));
    do_map = ismember(k, map_steps);

    link = compute_link(txPos, rxPos_array, cfg, arrays_cfg, ...
                        attitude_vec(k), sim, map_cfg, do_map);

    for u = 1:Nusers
        SNR_mat(k,u)    = link(u).SNR_dB;
        T_mat(k,u)      = link(u).T_Mbps;
        C_mat(k,u)      = link(u).capacity_Mbps;
        margin_mat(k,u) = link(u).margin_dB;
        txGain_mat(k,u) = link(u).txGain_dBi;
        linkOK_mat(k,u) = link(u).linkOK;
        dist_mat(k,u) = link(u).dist_m;
    end

    simData(k).txPos       = txPos;
    simData(k).rxPos_array = rxPos_array;
    simData(k).arrays_cfg  = arrays_cfg;
    simData(k).attitude    = attitude_vec(k);
    simData(k).link        = link;
    simData(k).t           = t_vec(k);

    if mod(k,20)==0, fprintf('  %d/%d\n', k, Ntime); end
end
fprintf('Done.\n\n');


%%  RESULTADOS


T_total = sum(T_mat, 2);
T_e2e   = min(T_total, backhaul.ul_Mbps);

fprintf('Access link OK:    %.1f%%\n', 100*mean(all(linkOK_mat,2)));
fprintf('Mean aggregate DL: %.1f Mbps\n', mean(T_total));
fprintf('Mean E2E (UL lim): %.1f Mbps\n\n', mean(T_e2e));

plot_results(t_vec, SNR_mat, T_mat, C_mat, txGain_mat, margin_mat, ...
             linkOK_mat, dist_mat, Nusers, sim.accessTech);

plot_trajectory_map(traj, rxPos_array, gDT, Nusers);


%%  VÍDEO  (descomentar para gerar)


% switch lower(TERRAIN_MODE)
%     case 'tif'
%         render_simulation_video_3(simData, cfg, gDT, terrain_renderer, ...
%             'OutputFile','uav_relay_terrain.mp4', ...
%             'NumLaps', 1, 'CameraMode', 'orbit');
%     case 'siteviewer'
%         render_simulation_video(simData, cfg, gDT, [], ...
%             'OutputFile','uav_relay_demo.mp4');
% end


%%  FUNÇÕES LOCAIS


function link = compute_link(txPos, rxPos_array, cfg, arrays_cfg, ...
                              drone_attitude, sim, map_cfg, do_map)
    if isempty(drone_attitude)
        drone_attitude = struct('roll_deg',0,'pitch_deg',0,'yaw_deg',0);
    end

    kT_dBm_perHz   = -174;
    noisePower_dBm = kT_dBm_perHz + 10*log10(cfg.BHz) + cfg.NF_dB;

    switch lower(sim.accessTech)
        case '5g'
            cfg_link.BHz      = cfg.BHz;
            cfg_link.SCS_Hz   = cfg.g5.SCS_Hz;
            cfg_link.n_layers = cfg.g5.n_layers;
            cfg_link.overhead = cfg.g5.overhead;
        case 'wifi6'
            cfg_link.BHz   = cfg.BHz;
            cfg_link.nSS   = cfg.wifi.nSS;
            cfg_link.GI_us = cfg.wifi.GI_us;
    end

    txPower_W = 10^((cfg.txPower_dBm - 30) / 10);

    sv_tx = txsite('Latitude',             txPos.lat, ...
                   'Longitude',            txPos.lon, ...
                   'AntennaHeight',        txPos.alt, ...
                   'TransmitterFrequency', cfg.fHz, ...
                   'TransmitterPower',     txPower_W);

    switch lower(sim.propModel)
        case 'longley-rice'; sv_pm = propagationModel('longley-rice');
        otherwise;           sv_pm = propagationModel('freespace');
    end

    N_users = numel(rxPos_array);
    link    = struct();

    for u = 1:N_users
        rxPos = rxPos_array(u);
        [az_deg, el_deg, dist_m] = geodetic_direction(txPos, rxPos);
        [txGain_dBi, ~] = antenna_gain(arrays_cfg, drone_attitude, az_deg, el_deg);

        sv_rx = rxsite('Latitude',            rxPos.lat, ...
                       'Longitude',           rxPos.lon, ...
                       'AntennaHeight',       rxPos.alt, ...
                       'ReceiverSensitivity', cfg.rxSens_dBm);

        Prx_iso = sigstrength(sv_rx, sv_tx, sv_pm);
        Prx_dBm = Prx_iso + txGain_dBi + cfg.rxGain_dBi;

        SNR_dB    = Prx_dBm - noisePower_dBm;
        C_Mbps    = cfg.BHz * log2(1 + 10^(SNR_dB/10)) / 1e6;
        margin_dB = Prx_dBm - (cfg.rxSens_dBm + cfg.fadeMargin_dB);

        switch lower(sim.accessTech)
            case '5g';    [T_Mbps, mcs, eta] = snr2throughput_5g(SNR_dB, cfg_link);
            case 'wifi6'; [T_Mbps, mcs, eta] = snr2throughput_wifi6(SNR_dB, cfg_link);
        end

        link(u).Prx_dBm       = Prx_dBm;
        link(u).txGain_dBi    = txGain_dBi;
        link(u).SNR_dB        = SNR_dB;
        link(u).capacity_Mbps = C_Mbps;
        link(u).T_Mbps        = T_Mbps;
        link(u).mcs_idx       = mcs;
        link(u).spectral_eff  = eta;
        link(u).margin_dB     = margin_dB;
        link(u).linkOK        = margin_dB >= 0;
        link(u).az_to_user    = az_deg;
        link(u).el_to_user    = el_deg;
        link(u).dist_m        = dist_m;
    end

    if do_map
        build_coverage_map(txPos, rxPos_array, cfg, cfg_link, arrays_cfg, ...
                           drone_attitude, sim, map_cfg, noisePower_dBm, ...
                           sv_tx, sv_pm);
    end
end


function build_coverage_map(txPos, rxPos_array, cfg, cfg_link, arrays_cfg, ...
                             drone_attitude, sim, map_cfg, noisePower_dBm, ...
                             sv_tx, sv_pm)
    N_users    = numel(rxPos_array);
    R_m        = map_cfg.MapRange;
    viewer_map = siteviewer('Basemap','satellite');

    show(sv_tx, 'Map', viewer_map);
    for u = 1:N_users
        sv_u = rxsite('Latitude',      rxPos_array(u).lat, ...
                      'Longitude',     rxPos_array(u).lon, ...
                      'AntennaHeight', rxPos_array(u).alt);
        show(sv_u, 'Map', viewer_map);
    end

    pd = sinr(sv_tx, sv_pm, ...
              'MaxRange',           R_m, ...
              'Resolution',         map_cfg.Resolution, ...
              'ReceiverNoisePower', noisePower_dBm);

    lat_pd  = pd.Data.Latitude;
    lon_pd  = pd.Data.Longitude;
    SINR_dB = pd.Data.SINR;

    metersPerDegLon = 111320 * cosd(txPos.lat);
    dX = (lon_pd - txPos.lon) * metersPerDegLon;
    dY = (lat_pd - txPos.lat) * 111320;
    az_pts = mod(rad2deg(atan2(dX, dY)), 360);
    el_pts = rad2deg(atan2(-txPos.alt, sqrt(dX.^2 + dY.^2)));

    G_pts = zeros(size(SINR_dB));
    for i = 1:numel(SINR_dB)
        [g,~] = antenna_gain(arrays_cfg, drone_attitude, az_pts(i), el_pts(i));
        G_pts(i) = g;
    end
    SINR_corr = SINR_dB + G_pts + cfg.rxGain_dBi;
    SINR_lin  = 10.^(SINR_corr / 10);

    switch lower(map_cfg.type)
        case 'snr'
            plotVar = SINR_corr; varName = 'SNR_dB'; clim_v = [-10 40];
        case 'shannon'
            plotVar = cfg.BHz * log2(1 + SINR_lin) / 1e6;
            varName = 'Capacity_Mbps'; clim_v = [0 300];
        case 'throughput'
            switch lower(sim.accessTech)
                case '5g';    plotVar = snr2throughput_5g(SINR_corr(:), cfg_link);
                case 'wifi6'; plotVar = snr2throughput_wifi6(SINR_corr(:), cfg_link);
            end
            varName = 'Throughput_Mbps'; clim_v = [0 150];
        otherwise
            plotVar = SINR_corr; varName = 'SNR_dB'; clim_v = [-10 40];
    end

    tbl    = table(lat_pd(:), lon_pd(:), plotVar(:), ...
                   'VariableNames', {'Latitude','Longitude', varName});
    pd_out = propagationData(tbl, 'DataVariableName', varName);
    contour(pd_out, 'Map', viewer_map, 'Type','custom', ...
            'ColorLimits', clim_v, 'Colormap', flipud(jet));
end


function [T_Mbps, mcs_idx, spectral_eff] = snr2throughput_5g(snr_dB, cfg_link)
    MCS_table = [
        2,120,0.2344; 2,157,0.3066; 2,193,0.3770; 2,251,0.4902;
        2,308,0.6016; 2,379,0.7402; 2,449,0.8770; 2,526,1.0273;
        2,602,1.1758; 2,679,1.3262; 4,340,1.3281; 4,378,1.4766;
        4,434,1.6953; 4,490,1.9141; 4,553,2.1602; 4,616,2.4063;
        4,658,2.5703; 6,438,2.5664; 6,466,2.7305; 6,517,3.0293;
        6,567,3.3223; 6,616,3.6094; 6,666,3.9023; 6,719,4.2129;
        6,772,4.5234; 6,822,4.8164; 6,873,5.1152; 6,910,5.3320; 6,948,5.5547;
    ];
    SNR_thresh = [-5.0;-3.5;-2.5;-1.0;0.0;1.5;3.0;4.5;5.5;6.5;6.5;7.5;
                   9.0;10.5;12.0;13.5;14.5;14.5;15.5;17.0;18.5;20.0;21.5;
                  23.0;24.5;26.0;27.5;28.5;29.5];
    PRB_table = [5,25,11,0; 10,52,24,11; 15,79,38,18; 20,106,51,24;
                 25,133,65,31; 30,160,78,38; 40,216,106,51; 50,270,133,65;
                 60,0,162,79; 80,0,217,107; 100,0,273,135];

    BW_MHz  = cfg_link.BHz/1e6;
    SCS_kHz = cfg_link.SCS_Hz/1e3;
    BW_row  = find(abs(PRB_table(:,1)-BW_MHz)<0.5, 1);
    switch SCS_kHz; case 15; col=2; case 30; col=3; case 60; col=4; end
    N_PRB           = PRB_table(BW_row, col);
    slots_per_sec   = (SCS_kHz/15)*1000;
    RE_per_PRB_slot = 168;
    bler_slope      = 1.4;

    snr_dB = snr_dB(:);
    N = numel(snr_dB);
    T_Mbps = zeros(N,1); mcs_idx = zeros(N,1); spectral_eff = zeros(N,1);
    for i = 1:N
        best_T=0; best_mcs=0; best_eta=0;
        for v = 1:size(MCS_table,1)
            BLER   = 1/(1+exp(bler_slope*(snr_dB(i)-SNR_thresh(v))));
            bits   = MCS_table(v,3)*N_PRB*RE_per_PRB_slot*(1-cfg_link.overhead)*cfg_link.n_layers;
            T_cand = bits*slots_per_sec*(1-BLER)/1e6;
            if T_cand>best_T; best_T=T_cand; best_mcs=v-1; best_eta=MCS_table(v,3); end
        end
        T_Mbps(i)=best_T; mcs_idx(i)=best_mcs; spectral_eff(i)=best_eta;
    end
end


function [T_Mbps, mcs_idx, spectral_eff] = snr2throughput_wifi6(snr_dB, cfg_link)
    MCS_table = [0,1,1,2,8.6; 1,2,1,2,17.2; 2,2,3,4,25.8; 3,4,1,2,34.4;
                 4,4,3,4,51.6; 5,6,2,3,68.8; 6,6,3,4,77.4; 7,6,5,6,86.0;
                 8,8,3,4,103.2; 9,8,5,6,114.7; 10,10,3,4,129.0; 11,10,5,6,143.4];
    SNR_thresh = [-1;5;8;11;15;18;20;22;26;28;31;34];
    bler_slope = 1.6;
    bw_scale   = cfg_link.BHz/20e6;
    gi_scale   = (12.8+0.8)/(12.8+cfg_link.GI_us);

    snr_dB = snr_dB(:);
    N = numel(snr_dB);
    T_Mbps = zeros(N,1); mcs_idx = zeros(N,1); spectral_eff = zeros(N,1);
    for i = 1:N
        best_T=0; best_mcs=0; best_eta=0;
        for v = 1:size(MCS_table,1)
            BLER   = 1/(1+exp(bler_slope*(snr_dB(i)-SNR_thresh(v))));
            T_cand = MCS_table(v,5)*bw_scale*gi_scale*cfg_link.nSS*(1-BLER);
            if T_cand>best_T
                best_T=T_cand; best_mcs=MCS_table(v,1);
                best_eta=MCS_table(v,2)*MCS_table(v,3)/MCS_table(v,4);
            end
        end
        T_Mbps(i)=best_T; mcs_idx(i)=best_mcs; spectral_eff(i)=best_eta;
    end
end


function [G_dBi, G_lin] = antenna_gain(arrays_cfg, drone_attitude, az_to_user_deg, el_to_user_deg)
    if isempty(drone_attitude)
        drone_attitude = struct('roll_deg',0,'pitch_deg',0,'yaw_deg',0);
    end
    roll  = deg2rad(drone_attitude.roll_deg);
    pitch = deg2rad(drone_attitude.pitch_deg);
    yaw   = deg2rad(drone_attitude.yaw_deg);
    R_roll  = [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
    R_pitch = [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
    R_yaw   = [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
    R_b2w   = R_yaw*R_pitch*R_roll;
    az_r = deg2rad(az_to_user_deg); el_r = deg2rad(el_to_user_deg);
    u_world = [sin(az_r)*cos(el_r); cos(az_r)*cos(el_r); sin(el_r)];
    N_arrays = numel(arrays_cfg); G_lin_all = zeros(1,N_arrays);
    for ii = 1:N_arrays
        ac = arrays_cfg(ii);
        az_bs = deg2rad(ac.boresight_az); el_bs = deg2rad(ac.boresight_el);
        bs_body  = [cos(el_bs)*sin(az_bs); -cos(el_bs)*cos(az_bs); sin(el_bs)];
        bs_world = R_b2w*bs_body; bs_world = bs_world/norm(bs_world);
        cos_psi  = max(-1,min(1,dot(bs_world,u_world)));
        switch lower(ac.type)
            case 'preset'; G_lin_all(ii) = gain_preset(ac,cos_psi);
            case 'lut';    G_lin_all(ii) = gain_lut(ac,bs_world,u_world);
            otherwise;     G_lin_all(ii) = 1;
        end
    end
    G_lin = max(G_lin_all); G_dBi = 10*log10(max(G_lin,1e-10));
end


function G_lin = gain_preset(ac, cos_psi)
    G_elem_lin = 10^(ac.G_elem_dBi/10);
    switch lower(ac.preset)
        case 'omni';   G_lin = G_elem_lin;
        case 'dipole'; G_lin = 1.64*G_elem_lin*max(0,1-cos_psi^2);
        case 'patch'
            if cos_psi<=0; G_lin=0; else; G_lin=G_elem_lin*cos_psi; end
        case 'yagi'
            if isfield(ac,'HPBW_half_deg') && ac.HPBW_half_deg>0
                n = log(0.5)/log(cosd(ac.HPBW_half_deg));
            else; n = log(0.5)/log(cosd(30));
            end
            if cos_psi<=0; G_lin=0; else; G_lin=G_elem_lin*cos_psi^n; end
        case 'array'
            if cos_psi<=0; G_lin=0; return; end
            if isfield(ac,'HPBW_half_deg') && ac.HPBW_half_deg>0
                n = log(0.5)/log(cosd(ac.HPBW_half_deg));
            else; n = log(0.5)/log(cosd(65));
            end
            N_total = ac.Nx*ac.Ny; eta = 1.0;
            if isfield(ac,'eta_element'); eta = eta*ac.eta_element; end
            if isfield(ac,'eta_array');   eta = eta*ac.eta_array; end
            G_lin = (cos_psi^n)*G_elem_lin*N_total*eta;
        otherwise; G_lin = G_elem_lin;
    end
end


function G_lin = gain_lut(ac, bs_world, u_world)
    persistent lut_cache;
    if isempty(lut_cache); lut_cache = containers.Map; end
    if ~isKey(lut_cache, ac.lut_file)
        try; lut_cache(ac.lut_file) = readmatrix(ac.lut_file);
        catch; warning('LUT não encontrada: %s', ac.lut_file); G_lin=1; return; end
    end
    lut = lut_cache(ac.lut_file);
    cos_psi  = max(-1,min(1,dot(bs_world,u_world)));
    el_query = 90-rad2deg(acos(cos_psi));
    gain_dBi = interp1(lut(:,2),lut(:,3),el_query,'linear',min(lut(:,3)));
    G_lin    = max(0,10^(gain_dBi/10));
end


function plot_results(t_vec, SNR_mat, T_mat, C_mat, txGain_mat, margin_mat, ...
                      linkOK_mat, dist_mat, Nusers, accessTech)
    T_total = sum(T_mat,2);
    C_total = sum(C_mat, 2);
    leg = arrayfun(@(u)sprintf('User %d',u), 1:Nusers, 'UniformOutput',false);

    figure('Name','SNR','Color','w');
    plot(t_vec,SNR_mat,'LineWidth',1.5); grid on;
    xlabel('Time (s)'); ylabel('SNR (dB)'); title('SNR per user');
    legend(leg,'Location','best');

    figure('Name','Throughput','Color','w');
    plot(t_vec,T_mat,'LineWidth',1.5); hold on;
    plot(t_vec,T_total,'k--','LineWidth',2,'DisplayName','Aggregate');
    grid on; xlabel('Time (s)'); ylabel('Throughput (Mbps)');
    title(sprintf('%s throughput per user',upper(accessTech)));
    legend([leg,{'Aggregate'}],'Location','best');

    figure('Name','Array gain','Color','w');
    plot(t_vec,txGain_mat,'LineWidth',1.5); grid on;
    xlabel('Time (s)'); ylabel('TX array gain (dBi)');
    title('Best panel gain per user'); legend(leg,'Location','best');

    figure('Name','Link margin','Color','w');
    plot(t_vec,margin_mat,'LineWidth',1.5); hold on;
    yline(0,'r--','LineWidth',1.5);
    grid on; xlabel('Time (s)'); ylabel('Margin (dB)');
    title('Link margin per user'); legend(leg,'Location','best');

    figure('Name','Availability','Color','w');
    imagesc(t_vec,1:Nusers,linkOK_mat');
    colormap([0.85 0.2 0.2; 0.2 0.75 0.35]);
    colorbar('Ticks',[0.25 0.75],'TickLabels',{'Fail','OK'});
    xlabel('Time (s)'); ylabel('User'); yticks(1:Nusers);
    title('Link availability per user');

    figure('Name','Shannon Capacity','Color','w');
    plot(t_vec, C_mat, 'LineWidth', 1.5); hold on;
    plot(t_vec, C_total, 'k--', 'LineWidth', 2, 'DisplayName', 'Aggregate');
    grid on;
    xlabel('Time (s)'); ylabel('Shannon Capacity (Mbps)');
    title('Theoretical channel capacity per user (Shannon)');
    legend([leg, {'Aggregate'}], 'Location', 'best');

    figure('Name','Distance to drone','Color','w');
    plot(t_vec, dist_mat/1000, 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Distance (km)');
    title('Distance from UAV to each user');
    legend(leg, 'Location','best');
end


function traj = generateDroneTrajectory(cfg)
    s = (0:cfg.step_m:cfg.totalDist_m).';
    switch lower(cfg.type)
        case 'circle'
            theta = s/cfg.radius_m;
            x = cfg.radius_m*cos(theta); y = cfg.radius_m*sin(theta);
        case 'line'
            x = s*cosd(cfg.azimuth_deg); y = s*sind(cfg.azimuth_deg);
        otherwise; error('Trajectory não suportada: %s', cfg.type);
    end
    traj.lat   = cfg.centerLat + y/111320;
    traj.lon   = cfg.centerLon + x./(111320*cosd(cfg.centerLat));
    traj.alt_m = cfg.alt_m .* ones(size(s));
    traj.N     = numel(s);
end


function users = generateUsersInArea(cLat, cLon, latSpan, lonSpan, N, alt)
    users.lat   = cLat + latSpan*(rand(N,1)-0.5);
    users.lon   = cLon + lonSpan*(rand(N,1)-0.5);
    users.alt_m = alt*ones(N,1);
    users.N     = N;
end

function plot_trajectory_map(traj, rxPos_array, gDT, Nusers)
    figure('Name','Trajectory Map','Color','w','Position',[100 100 700 600]);
    ax = axes('Parent', gcf);

    % --- Trajetória circular ---
    plot(ax, traj.lon, traj.lat, 'b-', 'LineWidth', 1.8, 'DisplayName','UAV Trajectory');
    hold(ax, 'on');

    % Marcador de início/fim da trajetória
    scatter(ax, traj.lon(1), traj.lat(1), 80, 'b', 'filled', ...
            'Marker','^', 'DisplayName','Start/End');

    % Centro
    plot(ax, gDT.centerLon, gDT.centerLat, 'k+', ...
         'MarkerSize', 12, 'LineWidth', 1.5, 'DisplayName','Centro');

    % --- Utilizadores ---
    colors_u = lines(Nusers);
    for u = 1:Nusers
        scatter(ax, rxPos_array(u).lon, rxPos_array(u).lat, 120, ...
                colors_u(u,:), 'filled', 'Marker','o', ...
                'HandleVisibility','off');
        text(ax, rxPos_array(u).lon, rxPos_array(u).lat, ...
             sprintf('  U%d', u), ...
             'FontSize', 10, 'FontWeight', 'bold', ...
             'Color', colors_u(u,:), ...
             'VerticalAlignment', 'middle');
    end

    % Entrada de legenda única para utilizadores
    scatter(ax, NaN, NaN, 120, [0.5 0.5 0.5], 'filled', 'Marker','o', ...
            'DisplayName','Users');

    % --- Decoração ---
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'Longitude (°)');
    ylabel(ax, 'Latitude (°)');
    title(ax, sprintf(' UAV Trajectory + Users  |  Alt: %.0f m MSL  |  R: %.0f m', ...
                      gDT.alt_m, gDT.radius_m));
    legend(ax, 'Location','best');

    % Raio em graus para anotação
    deg_per_m_lat = 1/111320;
    text(ax, gDT.centerLon, gDT.centerLat + gDT.radius_m*deg_per_m_lat*0.55, ...
         sprintf('r = %.0f m', gDT.radius_m), ...
         'FontSize', 9, 'Color', [0.3 0.3 0.8], ...
         'HorizontalAlignment','center');

    hold(ax, 'off');
end


function [az_deg, el_deg, dist_m] = geodetic_direction(from, to)
    R_earth = 6371000;
    lat1 = deg2rad(from.lat); lon1 = deg2rad(from.lon);
    lat2 = deg2rad(to.lat);   lon2 = deg2rad(to.lon);
    dlat = lat2-lat1; dlon = lon2-lon1;
    a        = sin(dlat/2)^2 + cos(lat1)*cos(lat2)*sin(dlon/2)^2;
    d_horiz  = R_earth*2*atan2(sqrt(a),sqrt(1-a));
    d_vert   = to.alt - from.alt;
    dist_m   = sqrt(d_horiz^2 + d_vert^2);
    el_deg   = rad2deg(atan2(d_vert, d_horiz));
    x_bear   = sin(dlon)*cos(lat2);
    y_bear   = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon);
    az_deg   = mod(rad2deg(atan2(x_bear,y_bear)), 360);
end


function s = merge_struct(s1, s2)
    s = s1; f = fieldnames(s2);
    for i = 1:numel(f); s.(f{i}) = s2.(f{i}); end
end