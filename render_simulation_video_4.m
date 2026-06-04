function render_simulation_video_4(simData, cfg, gDT, terrain, varargin)
% RENDER_SIMULATION_VIDEO_4  v7  –  Renderer Cinematográfico UAV Relay
%
%
% Uso básico:
%   render_simulation_video_4(simData, cfg, gDT, terrain)
%
% Batch (todos os ângulos de uma vez):
%   render_all_views(simData, cfg, gDT, terrain)
%
% Parâmetros:
%   'OutputFile'   – nome do ficheiro de saída        (padrão: 'uav_sim_v7.mp4')
%   'FrameRate'    – fps                              (padrão: 30)
%   'Quality'      – qualidade MPEG-4 [0-100]         (padrão: 95)
%   'CameraMode'   – ver lista abaixo                 (padrão: 'orbit_low')
%   'ShowBeams'    – mostrar cones de antena          (padrão: true)
%   'ShowHeatmap'  – mostrar mapa SNR                 (padrão: true)
%   'ShowOverlay'  – mostrar HUD de texto             (padrão: true)
%   'Resolution'   – [largura altura] em pixels       (padrão: [1920 1080])
%   'HeatmapRes'   – resolução da grelha SNR          (padrão: 80)
%   'BeamAlpha'    – transparência dos cones          (padrão: 0.18)
%   'NumLaps'      – repetições da trajectória        (padrão: 1)
%   'FadeFrames'   – frames de fade in/out            (padrão: 25)
%   'ZExag'        – exagero vertical do terreno      (padrão: 2.5)
%   'UserAnchor'   – índice utilizador para ground_human (padrão: 1)
%
% Modos de câmara:
%   'orbit_low'       – órbita próxima (vista geral principal)
%   'orbit'           – órbita clássica mais afastada
%   'dramatic_reveal' – abre do drone e recua revelando o terreno
%   'ground_human'    – perspectiva ao nível do solo de um utilizador
%   'fly_alongside'   – câmara voa ao lado do drone
%   'follow'          – segue por trás do drone
%   'top_down'        – vista aérea zenital (foco no heatmap)
%   'fixed'           – ângulo fixo clássico
%   'cinematic_auto'  – sequência automática que alterna modos

    %% ── Parser de argumentos ────────────────────────────────────────
    p = inputParser;
    addParameter(p,'OutputFile',    'uav_sim_v7.mp4');
    addParameter(p,'FrameRate',     30);
    addParameter(p,'Quality',       95);
    addParameter(p,'CameraMode',    'orbit_low');
    addParameter(p,'ShowBeams',     true);
    addParameter(p,'ShowHeatmap',   true);
    addParameter(p,'ShowOverlay',   true);
    addParameter(p,'Resolution',    [1920 1080]);
    addParameter(p,'HeatmapRes',    80);
    addParameter(p,'BeamAlpha',     0.18);
    addParameter(p,'NumLaps',       1);
    addParameter(p,'FadeFrames',    25);
    addParameter(p,'ZExag',         2.5);
    addParameter(p,'UserAnchor',    1);
    parse(p, varargin{:});
    opt    = p.Results;
    Z_EXAG = opt.ZExag;

    %% ── Repetir trajectória NumLaps vezes ──────────────────────────
    Ntime_orig = numel(simData);
    if opt.NumLaps > 1
        dt = 0;
        if Ntime_orig > 1; dt = simData(2).t - simData(1).t; end
        base    = simData;
        simData = repmat(base, 1, opt.NumLaps);
        for lap = 1:opt.NumLaps
            t_off = (lap-1)*Ntime_orig*dt;
            for k = (lap-1)*Ntime_orig+1 : lap*Ntime_orig
                simData(k).t = simData(k).t + t_off;
            end
        end
    end
    Ntime  = numel(simData);
    Nusers = numel(simData(1).rxPos_array);

    if ~isfield(cfg,'NF_dB') && isfield(cfg,'NF_dB_val')
        cfg.NF_dB = cfg.NF_dB_val;
    end
    if ~isfield(cfg,'rxGain_dBi'); cfg.rxGain_dBi = 0; end

    %% ── Factores de conversão graus → metros ────────────────────────
    mpdLat = 111320;
    mpdLon = 111320 * cosd(gDT.centerLat);

    %% ── Trajectória em metros cartesianos ───────────────────────────
    traj_x = arrayfun(@(s)(s.txPos.lon - gDT.centerLon)*mpdLon, simData);
    traj_y = arrayfun(@(s)(s.txPos.lat - gDT.centerLat)*mpdLat, simData);
    traj_z = arrayfun(@(s) s.txPos.alt, simData);

    %% ── Utilizadores em metros cartesianos ─────────────────────────
    user_x = arrayfun(@(u)(simData(1).rxPos_array(u).lon - gDT.centerLon)*mpdLon, 1:Nusers);
    user_y = arrayfun(@(u)(simData(1).rxPos_array(u).lat - gDT.centerLat)*mpdLat, 1:Nusers);

    %% ── Terreno em metros cartesianos ───────────────────────────────
    if ~isempty(terrain) && isfield(terrain,'X') && isfield(terrain,'Y') && isfield(terrain,'Z')
        T_X = (terrain.X - gDT.centerLon) * mpdLon;
        T_Y = (terrain.Y - gDT.centerLat) * mpdLat;
        T_Z =  terrain.Z;

        if any(isnan(T_Z(:)))
            [nr, nc] = size(T_Z);
            [ci, ri] = meshgrid(1:nc, 1:nr);
            valid    = ~isnan(T_Z);
            if sum(valid(:)) > 3
                T_Z = griddata(ci(valid), ri(valid), T_Z(valid), ci, ri, 'linear');
                T_Z(isnan(T_Z)) = nanmean(T_Z(:));
            else
                T_Z(isnan(T_Z)) = 0;
            end
        end

        lon_vec_m = unique(T_X(1,:))';
        lat_vec_m = unique(T_Y(:,1));
        [LON_nd, LAT_nd] = ndgrid(lon_vec_m, lat_vec_m);
        Z_nd = griddata(T_X(:), T_Y(:), T_Z(:), LON_nd, LAT_nd, 'linear');
        Z_nd(isnan(Z_nd)) = nanmean(T_Z(:));
        terrain_interp = griddedInterpolant(LON_nd, LAT_nd, Z_nd, 'linear', 'nearest');
        has_terrain    = true;

        fprintf('Terreno: [%d×%d]  X∈[%.0f,%.0f]m  Y∈[%.0f,%.0f]m  Z∈[%.0f,%.0f]m\n', ...
            size(T_Z,1),size(T_Z,2), ...
            min(T_X(:)),max(T_X(:)),min(T_Y(:)),max(T_Y(:)), ...
            min(T_Z(:)),max(T_Z(:)));
    else
        R = gDT.radius_m * 1.6;
        [T_X, T_Y] = meshgrid(linspace(-R,R,60), linspace(-R,R,60));
        T_Z        = zeros(size(T_X));
        terrain_interp = [];
        has_terrain    = false;
        fprintf('Sem terreno – usando plano plano.\n');
    end

    %% ── Altitude do utilizador âncora para câmara ground_human ─────
    u_anc = max(1, min(opt.UserAnchor, Nusers));
    if has_terrain
        user_gz = arrayfun(@(u) terrain_interp(user_x(u), user_y(u)), 1:Nusers);
    else
        user_gz = zeros(1, Nusers);
    end

    %% ── Grelha hexagonal decorativa ────────────────────────────────
    hex_grid = make_hex_grid(gDT.radius_m * 1.45, 160);

    %% ── Pré-calcular heatmaps SNR ───────────────────────────────────
    hRes = opt.HeatmapRes;
    R_hm = gDT.radius_m * 1.35;
    hx   = linspace(-R_hm, R_hm, hRes);
    hy   = linspace(-R_hm, R_hm, hRes);
    [HX, HY] = meshgrid(hx, hy);
    lambda   = 3e8 / cfg.fHz;
    N_dBm    = -174 + 10*log10(cfg.BHz) + cfg.NF_dB;

    if has_terrain
        gz_hm_grid = terrain_interp(HX', HY')';
        gz_hm_grid(~isfinite(gz_hm_grid)) = min(T_Z(:));
    else
        gz_hm_grid = zeros(hRes, hRes);
    end

    fprintf('Pré-calculando heatmaps (%d frames base)...\n', Ntime_orig);
    heatmap_base = zeros(hRes, hRes, Ntime_orig);
    for k = 1:Ntime_orig
        dx_k = traj_x(k); dy_k = traj_y(k); dz_k = traj_z(k);
        gz_hm = gz_hm_grid;
        DX    = HX - dx_k; DY = HY - dy_k; DZ = gz_hm - dz_k;
        dist  = max(1, sqrt(DX.^2 + DY.^2 + DZ.^2));
        FSPL  = 20*log10(4*pi*dist/lambda);
        az_hm = mod(rad2deg(atan2(DX, DY)), 360);
        el_hm = rad2deg(atan2(-dz_k + gz_hm, sqrt(DX.^2 + DY.^2)));
        att_k  = simData(k).attitude;
        acfg_k = simData(k).arrays_cfg;
        G_hm   = zeros(hRes, hRes);
        for r = 1:hRes
            for c = 1:hRes
                [g,~] = antenna_gain_local(acfg_k, att_k, az_hm(r,c), el_hm(r,c));
                G_hm(r,c) = g;
            end
        end
        heatmap_base(:,:,k) = cfg.txPower_dBm + G_hm + cfg.rxGain_dBi - FSPL - N_dBm;
    end
    get_hm = @(k) heatmap_base(:,:, mod(k-1, Ntime_orig)+1);
    fprintf('Heatmaps prontos.\n');

    %% ── BeamAlpha adaptativo por modo ───────────────────────────────
    beam_alpha = opt.BeamAlpha;
    switch lower(opt.CameraMode)
        case {'ground_human','fly_alongside','follow'}
            beam_alpha = min(0.55, opt.BeamAlpha * 2.2);
        case {'top_down'}
            beam_alpha = max(0.05, opt.BeamAlpha * 0.5);
    end

    %% ── Drone size adaptativo por modo ─────────────────────────────
    % Tamanho físico base: ~20 m de raio máximo independentemente do radius_m.
    % Modos de câmara próximos usam escala menor (drone parece certo em close-up).
    drone_scale = 1.0;
    switch lower(opt.CameraMode)
        case {'ground_human'};           drone_scale = 0.45;  % câmara perto → drone real
        case {'fly_alongside','follow'}; drone_scale = 0.60;  % câmara semi-perto
        case {'top_down'};               drone_scale = 0.55;  % zenital → pode ser ligeiramente menor
        case {'dramatic_reveal'};        drone_scale = 0.80;  % revela de longe → um pouco maior
    end

    %% ── Figura e VideoWriter ────────────────────────────────────────
    fig = figure('Color','k', ...
                 'Position',[50 50 opt.Resolution(1) opt.Resolution(2)], ...
                 'MenuBar','none','ToolBar','none','Resize','off');
    set(fig,'Renderer','opengl');

    vw = VideoWriter(opt.OutputFile,'MPEG-4');
    vw.FrameRate = opt.FrameRate;
    vw.Quality   = opt.Quality;
    open(vw);

    user_pal = [0.20 0.80 1.00;
                1.00 0.55 0.10;
                0.35 1.00 0.45;
                1.00 0.25 0.55];
    if Nusers > 4; user_pal = [user_pal; lines(Nusers-4)]; end

    fprintf('Rendering %d frames → %s  [modo: %s]\n', ...
            Ntime, opt.OutputFile, opt.CameraMode);

    %% ── Loop principal ──────────────────────────────────────────────
    for k = 1:Ntime
        clf(fig);
        ax = axes('Parent',fig,'Position',[0 0 1 1],'Color','k');
        hold(ax,'on');

        dx_d = traj_x(k);
        dy_d = traj_y(k);
        dz_d = traj_z(k);
        SNR_k = get_hm(k);
        R_b2w = attitude_to_rotation(simData(k).attitude);
        prog  = (k-1) / max(Ntime-1, 1);

        %% 1+2. TERRENO COM HEATMAP ──────────────────────────────────
        if opt.ShowHeatmap
            snr_grid_interp = griddedInterpolant({hx,hy}, SNR_k', 'linear','nearest');
            SNR_on_T = snr_grid_interp(T_X, T_Y);
            surf(ax, T_X, T_Y, T_Z*Z_EXAG, SNR_on_T, ...
                 'EdgeColor','none','FaceAlpha',1.0, ...
                 'AmbientStrength',0.40,'DiffuseStrength',0.65,'SpecularStrength',0.08);
            colormap(ax, coverage_colormap());
            clim(ax, [-5 32]);
            cb = colorbar(ax,'Location','eastoutside', ...
                          'Color',[0.65 0.78 0.90],'FontSize',9,'FontName','Monospaced');
            cb.Label.String = 'SNR (dB)';
            cb.Label.Color  = [0.65 0.78 0.90];
            cb.Label.FontSize = 10;
            contour3(ax, T_X, T_Y, T_Z*Z_EXAG, SNR_on_T, [0  0 ], ...
                     'Color',[1.0 0.25 0.15],'LineWidth',1.5);
            contour3(ax, T_X, T_Y, T_Z*Z_EXAG, SNR_on_T, [10 10], ...
                     'Color',[0.25 1.0 0.45],'LineWidth',1.0);
        else
            snr_grid_interp = griddedInterpolant({hx,hy}, SNR_k', 'linear','nearest');
            surf(ax, T_X, T_Y, T_Z*Z_EXAG, ...
                 'FaceColor',[0.13 0.15 0.10],'EdgeColor','none','FaceAlpha',1.0, ...
                 'AmbientStrength',0.40,'DiffuseStrength',0.65,'SpecularStrength',0.08);
        end

        %% 3. GRELHA HEXAGONAL ───────────────────────────────────────
        for hi = 1:numel(hex_grid)
            hx_pts = hex_grid(hi).x;
            hy_pts = hex_grid(hi).y;
            cx_hex = mean(hx_pts);
            cy_hex = mean(hy_pts);
            snr_hex = snr_grid_interp(cx_hex, cy_hex);
            a_hex   = 0.04 + 0.09*min(1,max(0,(snr_hex+5)/37));
            if has_terrain
                z_hex = terrain_interp(cx_hex, cy_hex)*Z_EXAG + 3;
            else
                z_hex = 3;
            end
            plot3(ax, hx_pts, hy_pts, repmat(z_hex,size(hx_pts)), ...
                  'Color',[0.22 0.55 0.95 a_hex],'LineWidth',0.4);
        end

        %% 4. TRAJECTÓRIA COM GLOW ───────────────────────────────────
        t_len = min(k, round(Ntime_orig*0.28));
        t_idx = max(1,k-t_len):k;
        f_idx = k:min(Ntime, k+round(Ntime_orig*0.12));

        if has_terrain
            shadow_z = arrayfun(@(i) terrain_interp(traj_x(i),traj_y(i))*Z_EXAG+0.5, t_idx);
        else
            shadow_z = repmat(0.5, size(t_idx));
        end
        plot3(ax, traj_x(t_idx), traj_y(t_idx), shadow_z, ...
              'Color',[0.25 0.45 0.80 0.10],'LineWidth',2);
        plot3(ax, traj_x(t_idx), traj_y(t_idx), traj_z(t_idx), ...
              'Color',[0.30 0.72 1.0 0.18],'LineWidth',7);
        plot3(ax, traj_x(t_idx), traj_y(t_idx), traj_z(t_idx), ...
              'Color',[0.50 0.88 1.0],'LineWidth',1.8);
        plot3(ax, traj_x(f_idx), traj_y(f_idx), traj_z(f_idx), ...
              'Color',[0.30 0.62 0.85 0.10],'LineWidth',0.8,'LineStyle',':');

        if has_terrain
            gz_drone = terrain_interp(dx_d, dy_d)*Z_EXAG;
        else
            gz_drone = 0;
        end
        plot3(ax, [dx_d dx_d],[dy_d dy_d],[gz_drone dz_d], ...
              'Color',[0.35 0.55 0.85 0.18],'LineWidth',0.8,'LineStyle','--');

        %% 5. DRONE (asa fixa) ───────────────────────────────────────
        % Meia-envergadura: escala com o raio mas nunca ultrapassa 20 m.
        dr = min(gDT.radius_m * 0.020, 20) * drone_scale;

        % Direcção de voo a partir de ±2 frames de trajectória
        k_dn = min(k+2, Ntime);
        k_dp = max(k-2, 1);
        hdg_drone = [traj_x(k_dn)-traj_x(k_dp); traj_y(k_dn)-traj_y(k_dp); 0];
        if norm(hdg_drone) < 1e-6; hdg_drone = [1;0;0]; end
        hdg_drone = hdg_drone / norm(hdg_drone);

        show_halo = ismember(lower(opt.CameraMode), {'ground_human','fly_alongside'});
        draw_fixed_wing_drone(ax, [dx_d;dy_d;dz_d], hdg_drone, dr, ...
            [1.0 0.72 0.05], [0.90 0.82 0.12], show_halo);

        %% 6. BEAMS ──────────────────────────────────────────────────
        if opt.ShowBeams
            for ii = 1:numel(simData(k).arrays_cfg)
                ac      = simData(k).arrays_cfg(ii);
                az_bs   = deg2rad(ac.boresight_az);
                el_bs   = deg2rad(ac.boresight_el);
                bs_body = [cos(el_bs)*sin(az_bs); -cos(el_bs)*cos(az_bs); sin(el_bs)];
                bs_w    = R_b2w * bs_body;
                bs_w    = bs_w / norm(bs_w);
                G_lin   = gain_preset_local(ac, 1.0);
                G_dBi   = 10*log10(max(G_lin,1e-10));
                c_len   = gDT.radius_m * 0.50 * min(1,(G_dBi+5)/28);
                h_ang   = 30;
                if isfield(ac,'HPBW_half_deg'); h_ang = ac.HPBW_half_deg; end
                draw_beam_cone(ax, [dx_d dy_d dz_d], bs_w, c_len, h_ang, ...
                               beam_alpha, ii);
            end
        end

        %% 7. UTILIZADORES ───────────────────────────────────────────
        ring_r     = gDT.radius_m * 0.07;
        theta_ring = linspace(0, 2*pi, 40);
        is_ground  = strcmpi(opt.CameraMode, 'ground_human');

        for u = 1:Nusers
            lnk  = simData(k).link(u);
            ux   = user_x(u);
            uy   = user_y(u);
            ucol = user_pal(min(u,size(user_pal,1)),:);
            uz   = user_gz(u)*Z_EXAG + 1.5;

            t_norm   = min(1,max(0,lnk.T_Mbps/150));
            face_col = lnk.linkOK * min(1, ucol*(0.55+0.45*t_norm)) + ...
                       ~lnk.linkOK * [0.90 0.12 0.12];

            plot3(ax, ux+ring_r*cos(theta_ring), uy+ring_r*sin(theta_ring), ...
                  repmat(uz+0.3,1,40), 'Color',[ucol 0.30],'LineWidth',1.2);
            scatter3(ax, ux, uy, uz+1.0, 90, face_col, 'o','filled', ...
                     'MarkerEdgeColor','w','LineWidth',1.2);

            % Pilar vertical por baixo do marcador
            plot3(ax, [ux ux],[uy uy],[user_gz(u)*Z_EXAG, uz+1.0], ...
                  'Color',[ucol 0.20],'LineWidth',0.7,'LineStyle',':');

            if lnk.linkOK
                lbl = sprintf('U%d  %.0f Mb', u, lnk.T_Mbps);
                lc  = [0.88 0.92 0.96];
            else
                lbl = sprintf('U%d  NO LINK', u);
                lc  = [1.0 0.38 0.38];
            end

            % Offset vertical do label adaptativo ao modo de câmara
            if is_ground
                lbl_z_off = 25;
            else
                lbl_z_off = 80;
            end

            text(ax, ux+ring_r*1.5, uy+ring_r*1.5, uz+lbl_z_off, lbl, ...
                 'Color',lc,'FontSize',8.5,'FontWeight','bold', ...
                 'FontName','Monospaced','BackgroundColor',[0 0 0 0.50], ...
                 'EdgeColor',[ucol 0.40],'Margin',2);
        end

        %% 8. CÂMARA ─────────────────────────────────────────────────
        set_camera_v7(ax, opt.CameraMode, k, Ntime, prog, ...
                      dx_d, dy_d, dz_d, ...
                      traj_x, traj_y, traj_z, ...
                      user_x, user_y, user_gz, Z_EXAG, ...
                      u_anc, gDT.radius_m, has_terrain, terrain_interp);

        %% 9. ILUMINAÇÃO E ESTÉTICA ──────────────────────────────────
        light(ax,'Position',[0 0 dz_d*2.5],'Style','infinite','Color',[0.50 0.60 1.00]);
        light(ax,'Position',[gDT.radius_m -gDT.radius_m dz_d],'Style','local','Color',[0.18 0.08 0.0]);
        light(ax,'Position',[-gDT.radius_m*0.5 -gDT.radius_m*0.5 20],'Style','local','Color',[0.06 0.04 0.12]);
        lighting(ax,'gouraud'); material(ax,'shiny');

        set(ax,'Color','k', ...
               'GridColor',[0.11 0.14 0.19],'GridAlpha',0.65, ...
               'XColor',[0.18 0.20 0.25], ...
               'YColor',[0.18 0.20 0.25], ...
               'ZColor',[0.18 0.20 0.25]);
        grid(ax,'on');

        z_floor = min(T_Z(:))*Z_EXAG - 20;
        ax.ZLim = [z_floor, dz_d*1.35];
        ax.XLim = [-gDT.radius_m*1.58,  gDT.radius_m*1.58];
        ax.YLim = [-gDT.radius_m*1.58,  gDT.radius_m*1.58];
        set(ax,'DataAspectRatio',[1 1 1]);
        ax.XTickLabel = {}; ax.YTickLabel = {}; ax.ZTickLabel = {};
        ax.Projection = 'perspective';

        %% 10. OVERLAY ───────────────────────────────────────────────
        if opt.ShowOverlay
            delete(findall(fig,'Type','annotation'));
            draw_overlay_v7(fig, simData(k), k, Ntime, cfg, ...
                            opt.NumLaps, Ntime_orig, opt.CameraMode);
        end

        %% 11. FADE IN/OUT ───────────────────────────────────────────
        fade = compute_fade(k, Ntime, opt.FadeFrames);
        if fade < 0.999
            annotation(fig,'rectangle',[0 0 1 1], ...
                       'Color','none','FaceColor','k', ...
                       'FaceAlpha',1-fade,'LineStyle','none');
        end

        drawnow limitrate;
        writeVideo(vw, getframe(fig));
        if mod(k,20)==0
            fprintf('  Frame %d/%d  (%.0f%%)  [%s]\n', ...
                    k, Ntime, 100*k/Ntime, opt.CameraMode);
        end
    end

    close(vw); close(fig);
    fprintf('Vídeo guardado: %s\n', opt.OutputFile);
end


%% ══════════════════════════════════════════════════════════════════════
%  BATCH: produz todos os vídeos de uma vez
%% ══════════════════════════════════════════════════════════════════════

function render_all_views(simData, cfg, gDT, terrain, varargin)
% RENDER_ALL_VIEWS  Produz os 6 vídeos cinematográficos em sequência.
%
%   render_all_views(simData, cfg, gDT, terrain)
%   render_all_views(simData, cfg, gDT, terrain, 'Prefix','proj_', 'ZExag',2.5)
%
% Parâmetros extra repassados para render_simulation_video_4:
%   'Prefix'    – prefixo dos nomes de ficheiro   (padrão: 'uav_')
%   'ZExag'     – exagero vertical                (padrão: 2.5)
%   'FrameRate' – fps                             (padrão: 30)
%   'Quality'   – qualidade MPEG-4               (padrão: 95)
%   'NumLaps'   – repetições                     (padrão: 1)

    pp = inputParser;
    addParameter(pp,'Prefix',    'uav_');
    addParameter(pp,'ZExag',     2.5);
    addParameter(pp,'FrameRate', 30);
    addParameter(pp,'Quality',   95);
    addParameter(pp,'NumLaps',   1);
    parse(pp, varargin{:});
    po = pp.Results;

    common = {'ZExag',     po.ZExag, ...
              'FrameRate', po.FrameRate, ...
              'Quality',   po.Quality, ...
              'NumLaps',   po.NumLaps};

    % ── Tabela de vídeos a produzir ──────────────────────────────────
    %   {sufixo_ficheiro,  CameraMode,        ShowBeams, ShowHeatmap, NumLaps_factor}
    jobs = {
        'reveal',     'dramatic_reveal',  true,  true,  1;
        'overview',   'orbit_low',        true,  true,  1;
        'ground',     'ground_human',     true,  true,  1;
        'alongside',  'fly_alongside',    true,  false, 1;
        'heatmap',    'top_down',         false, true,  1;
        'follow',     'follow',           true,  true,  1;
    };

    fprintf('\n══════════════════════════════════════\n');
    fprintf(' BATCH: %d vídeos a produzir\n', size(jobs,1));
    fprintf('══════════════════════════════════════\n\n');

    for j = 1:size(jobs,1)
        suffix  = jobs{j,1};
        mode    = jobs{j,2};
        beams   = jobs{j,3};
        hmap    = jobs{j,4};
        out_f   = [po.Prefix suffix '.mp4'];

        fprintf('[%d/%d] %s → %s\n', j, size(jobs,1), mode, out_f);
        render_simulation_video_4(simData, cfg, gDT, terrain, ...
            'OutputFile',  out_f, ...
            'CameraMode',  mode, ...
            'ShowBeams',   beams, ...
            'ShowHeatmap', hmap, ...
            common{:});
        fprintf('  ✓ Concluído: %s\n\n', out_f);
    end

    fprintf('══════════════════════════════════════\n');
    fprintf(' BATCH concluído. %d ficheiros produzidos.\n', size(jobs,1));
    fprintf('══════════════════════════════════════\n');
end


%% ══════════════════════════════════════════════════════════════════════
%  CÂMARA v7  –  todos os modos num único lugar
%% ══════════════════════════════════════════════════════════════════════

function set_camera_v7(ax, mode, k, Ntime, prog, ...
                       dx, dy, dz, tx, ty, tz, ...
                       user_x, user_y, user_gz, Z_EXAG, ...
                       u_anc, radius, has_terrain, terrain_interp)

    % Easing cúbico suave (sem saltos no início/fim)
    ease = @(t) t.*t.*(3 - 2.*t);
    ep   = ease(prog);

    switch lower(mode)

        % ── ORBIT_LOW: órbita próxima com FOV largo ──────────────────
        case 'orbit_low'
            az   = 25 + prog*360;
            el   = 18 + 8*sin(2*pi*prog*2);
            dist = radius * 1.65;
            cx   = dx + dist*cosd(az)*cosd(el);
            cy   = dy + dist*sind(az)*cosd(el);
            cz   = dz*0.30 + dist*sind(el);
            campos(ax, [cx, cy, cz]);
            camtarget(ax, [dx*0.08, dy*0.08, dz*0.18]);
            camva(ax, 70);

        % ── ORBIT: órbita clássica afastada ─────────────────────────
        case 'orbit'
            az   = 35 + prog*52;
            el   = 27 + 9*sin(2*pi*prog);
            dist = radius * 3.0;
            campos(ax, [dist*cosd(az)*cosd(el), ...
                        dist*sind(az)*cosd(el), ...
                        dist*sind(el)+dz*0.22]);
            camtarget(ax, [0 0 dz*0.15]);
            camva(ax, 50);

        % ── DRAMATIC_REVEAL: abre do drone, recua revelando espaço ──
        %
        % Fase 0–40%: câmara muito perto do drone, quase ao nível dele
        % Fase 40–70%: recua e sobe devagar (revela terreno)
        % Fase 70–100%: órbita lenta para mostrar o sistema completo
        case 'dramatic_reveal'
            if prog < 0.40
                t_local  = ease(prog / 0.40);
                dist     = radius*(0.12 + 0.30*t_local);
                az       = 195 + 15*t_local;
                el_cam   = 5  + 12*t_local;
                target_z = dz * (0.85 + 0.15*t_local);
                fov      = 72 - 5*t_local;
            elseif prog < 0.70
                t_local  = ease((prog-0.40) / 0.30);
                dist     = radius*(0.42 + 1.40*t_local);
                az       = 210 + 10*t_local;
                el_cam   = 17 + 20*t_local;
                target_z = dz*(1.0 - 0.7*t_local);
                fov      = 67 - 12*t_local;
            else
                t_local  = ease((prog-0.70) / 0.30);
                dist     = radius*(1.82 + 0.25*sin(pi*t_local));
                az       = 220 + 35*t_local;
                el_cam   = 37 - 5*sin(pi*t_local);
                target_z = dz * 0.18;
                fov      = 55 + 5*t_local;
            end
            cx = dx + dist*cosd(az)*cosd(el_cam);
            cy = dy + dist*sind(az)*cosd(el_cam);
            cz = dz + dist*sind(el_cam);
            % Nunca ir abaixo do terreno
            if has_terrain
                gz_cam = terrain_interp(cx, cy)*Z_EXAG;
                cz     = max(cz, gz_cam + 8);
            end
            campos(ax, [cx, cy, cz]);
            camtarget(ax, [dx*0.15, dy*0.15, target_z]);
            camva(ax, fov);

        % ── GROUND_HUMAN: perspectiva humana ao nível do solo ────────
        %
        % A câmara fica junto ao utilizador âncora a 1.7m de altura.
        % Roda suavemente ao redor do utilizador, sempre a olhar para o drone.
        case 'ground_human'
            ux = user_x(u_anc);
            uy = user_y(u_anc);

            % Pequena órbita horizontal em volta do utilizador para mais dinamismo
            orbit_r  = radius * 0.045;
            orbit_az = prog * 60;   % percorre 60° ao longo de todo o vídeo
            cx = ux + orbit_r * cosd(orbit_az);
            cy = uy + orbit_r * sind(orbit_az);

            % Altitude base: terreno no ponto da câmara (não no utilizador âncora)
            % Isto evita clipping quando o utilizador está numa encosta e a órbita
            % leva a câmara para terreno mais alto ou mais baixo.
            if has_terrain
                gz_cam_ground = terrain_interp(cx, cy) * Z_EXAG;
            else
                gz_cam_ground = 0;
            end
            uz = gz_cam_ground + 1.7;   % altura dos olhos acima do terreno local

            campos(ax, [cx, cy, uz]);
            camtarget(ax, [dx, dy, dz]);
            camva(ax, 74);   % FOV largo = sensação humana imersiva

        % ── FLY_ALONGSIDE: câmara ao lado do drone em voo ────────────
        %
        % Offset lateral e ligeiramente atrás, a acompanhar velocidade.
        % Olha para a frente da trajectória do drone.
        case 'fly_alongside'
            k_next = min(k+4, Ntime);
            k_prev = max(k-2, 1);
            hdg = [tx(k_next)-tx(k_prev); ty(k_next)-ty(k_prev); 0];
            if norm(hdg) < 1e-6; hdg = [1;0;0]; end
            hdg  = hdg / norm(hdg);
            perp = cross(hdg, [0;0;1]);   % lado direito

            % Alterna suavemente entre esquerda e direita a cada meia volta
            side_sign = sign(sin(2*pi*prog));
            if side_sign == 0; side_sign = 1; end

            side_dist = radius * 0.38 * side_sign;
            back_dist = radius * 0.18;
            alt_off   = radius * 0.06;

            cam_pos = [dx; dy; dz] + ...
                      perp    * side_dist + ...
                      (-hdg)  * back_dist + ...
                      [0;0;1] * alt_off;

            % Nunca abaixo do terreno
            if has_terrain
                gz_cam = terrain_interp(cam_pos(1), cam_pos(2))*Z_EXAG;
                cam_pos(3) = max(cam_pos(3), gz_cam + 12);
            end

            % Olha para 60m à frente do drone na trajectória
            look_ahead = [dx;dy;dz] + hdg*60;
            campos(ax, cam_pos');
            camtarget(ax, look_ahead');
            camva(ax, 58);

        % ── FOLLOW: segue por trás do drone ─────────────────────────
        case 'follow'
            k_p = max(1,k-1);
            hx_v = dx-tx(k_p); hy_v = dy-ty(k_p);
            if norm([hx_v hy_v])<1e-6; hx_v=1; hy_v=0; end
            hdg = [hx_v;hy_v;0]/norm([hx_v;hy_v;0]);
            off = -hdg*radius*0.85;
            cam_pos = [dx+off(1), dy+off(2), dz*0.52];
            if has_terrain
                gz_cam = terrain_interp(cam_pos(1), cam_pos(2))*Z_EXAG;
                cam_pos(3) = max(cam_pos(3), gz_cam + 10);
            end
            campos(ax, cam_pos);
            camtarget(ax, [dx dy dz]);
            camva(ax, 60);

        % ── TOP_DOWN: vista zenital para foco no heatmap ─────────────
        case 'top_down'
            % Sobe suavemente até ter a área toda enquadrada
            height = radius * 2.2 + dz * 0.5;
            % Leve inclinação lateral para não ser completamente plano
            lean = radius * 0.25 * sin(2*pi*prog*0.5);
            campos(ax, [lean, 0, height]);
            camtarget(ax, [0, 0, 0]);
            camva(ax, 62);

        % ── FIXED: ângulo clássico estático ─────────────────────────
        case 'fixed'
            campos(ax, [radius*2.4, radius*1.9, dz*1.15]);
            camtarget(ax, [0 0 dz*0.14]);
            camva(ax, 48);

        % ── CINEMATIC_AUTO: sequência automática ─────────────────────
        %
        % Divide o vídeo em 4 actos e alterna automaticamente entre modos.
        % Bom para um único vídeo de apresentação completo.
        case 'cinematic_auto'
            % Acto 1 (0–20%):   reveal dramático (abre perto do drone)
            % Acto 2 (20–50%):  acompanha ao lado
            % Acto 3 (50–75%):  perspectiva humana
            % Acto 4 (75–100%): órbita geral a revelar cobertura
            if prog < 0.20
                % Reveal: perto e a subir
                t_l  = ease(prog/0.20);
                dist = radius*(0.15 + 0.55*t_l);
                az   = 200; el_cam = 8 + 24*t_l;
                cx = dx + dist*cosd(az)*cosd(el_cam);
                cy = dy + dist*sind(az)*cosd(el_cam);
                cz = dz + dist*sind(el_cam);
                if has_terrain
                    gz_c = terrain_interp(cx,cy)*Z_EXAG;
                    cz   = max(cz, gz_c+8);
                end
                campos(ax,[cx,cy,cz]);
                camtarget(ax,[dx*0.1,dy*0.1,dz*0.8]);
                camva(ax, 70 - 15*t_l);
            elseif prog < 0.50
                % Ao lado
                t_l  = ease((prog-0.20)/0.30);
                k_n  = min(k+4,Ntime); k_p2 = max(k-2,1);
                hdg2 = [tx(k_n)-tx(k_p2);ty(k_n)-ty(k_p2);0];
                if norm(hdg2)<1e-6; hdg2=[1;0;0]; end
                hdg2=hdg2/norm(hdg2);
                perp2=cross(hdg2,[0;0;1]);
                cp2 = [dx;dy;dz]+perp2*radius*0.40-hdg2*radius*0.20+[0;0;1]*radius*0.06;
                if has_terrain
                    gz_c=terrain_interp(cp2(1),cp2(2))*Z_EXAG;
                    cp2(3)=max(cp2(3),gz_c+12);
                end
                campos(ax,cp2');
                camtarget(ax,[dx+hdg2(1)*50,dy+hdg2(2)*50,dz]);
                camva(ax,58);
            elseif prog < 0.75
                % Ground human
                ux2=user_x(u_anc); uy2=user_y(u_anc);
                t_l=(prog-0.50)/0.25;
                orb_az2=t_l*50;
                orb_r2=radius*0.04;
                cx2_g = ux2+orb_r2*cosd(orb_az2);
                cy2_g = uy2+orb_r2*sind(orb_az2);
                if has_terrain
                    gz2_g = terrain_interp(cx2_g, cy2_g)*Z_EXAG;
                else
                    gz2_g = 0;
                end
                uz2 = gz2_g + 1.7;
                campos(ax,[cx2_g, cy2_g, uz2]);
                camtarget(ax,[dx,dy,dz]);
                camva(ax,74);
            else
                % Órbita final
                t_l =(prog-0.75)/0.25;
                az2 =210+ease(t_l)*120;
                el2 =22+12*sin(pi*t_l);
                dist2=radius*1.7;
                cx2=dx+dist2*cosd(az2)*cosd(el2);
                cy2=dy+dist2*sind(az2)*cosd(el2);
                cz2=dz*0.25+dist2*sind(el2);
                campos(ax,[cx2,cy2,cz2]);
                camtarget(ax,[0,0,dz*0.15]);
                camva(ax,68);
            end

        otherwise
            % Fallback: orbit_low
            az   = 25 + prog*360;
            el   = 18;
            dist = radius * 1.65;
            campos(ax, [dx+dist*cosd(az)*cosd(el), ...
                        dy+dist*sind(az)*cosd(el), ...
                        dz*0.30+dist*sind(el)]);
            camtarget(ax, [0 0 dz*0.18]);
            camva(ax, 70);
    end

    camup(ax, [0 0 1]);
end


%% ══════════════════════════════════════════════════════════════════════
%  OVERLAY v7  –  inclui modo de câmara no HUD
%% ══════════════════════════════════════════════════════════════════════

function draw_overlay_v7(fig, fd, k, Ntime, cfg, nlaps, N0, cam_mode)
    Nu      = numel(fd.link);
    T_total = sum([fd.link.T_Mbps]);
    N_ok    = sum([fd.link.linkOK]);
    G_best  = max([fd.link.txGain_dBi]);
    lap     = ceil(k/N0);

    s = sprintf(' t=%6.1fs   step %d/%d\n Alt: %4.0fm   Users: %d/%d OK\n DL: %5.1f Mbps   Gain: %.1f dBi ', ...
                fd.t, k, Ntime, fd.txPos.alt, N_ok, Nu, T_total, G_best);
    annotation(fig,'textbox',[0.005 0.902 0.310 0.086], ...
               'String',s,'Color',[0.80 0.92 1.00], ...
               'BackgroundColor',[0.02 0.03 0.07 0.75], ...
               'EdgeColor',[0.18 0.48 0.90],'FontSize',9, ...
               'FontName','Monospaced','FontWeight','bold', ...
               'FitBoxToText','off','LineWidth',1.0,'Margin',4);

    % Painel direito: inclui agora o modo de câmara
    cam_label = upper(strrep(cam_mode,'_',' '));
    annotation(fig,'textbox',[0.745 0.902 0.250 0.086], ...
               'String',sprintf(' UAV RELAY\n %s | %s\n [%s]  Lap %d/%d ', ...
                   upper(cfg.accessTech), upper(cfg.propModel), ...
                   cam_label, lap, nlaps), ...
               'Color',[0.22 0.82 1.00], ...
               'BackgroundColor',[0.02 0.03 0.07 0.75], ...
               'EdgeColor',[0.12 0.52 1.00],'FontSize',8.5, ...
               'FontName','Monospaced','FontWeight','bold', ...
               'FitBoxToText','off','HorizontalAlignment','center','Margin',4);

    % Barra de progresso
    prog = (k-1)/max(Ntime-1,1);
    annotation(fig,'rectangle',[0.005 0.010 0.990 0.013], ...
               'FaceColor',[0.06 0.08 0.12],'EdgeColor',[0.12 0.16 0.22],'LineWidth',0.5);
    if prog > 0.001
        annotation(fig,'rectangle',[0.005 0.010 max(0.003,0.990*prog) 0.013], ...
                   'FaceColor',[0.12 0.70 1.00],'EdgeColor','none');
    end
    for li = 1:nlaps-1
        lp = li*N0/max(Ntime-1,1);
        annotation(fig,'line',[0.005+lp*0.990, 0.005+lp*0.990],[0.008 0.025], ...
                   'Color',[0.8 0.8 0.8 0.45],'LineWidth',0.8);
    end
end


%% ══════════════════════════════════════════════════════════════════════
%  AUXILIARES (inalterados de v6 — só consolidados aqui)
%% ══════════════════════════════════════════════════════════════════════

function draw_fixed_wing_drone(ax, pos, hdg, scale, col_body, col_wing, show_halo)
% DRAW_FIXED_WING_DRONE  Desenha UAV de asa fixa estilizado.
%
%   pos       – [3×1] posição no mundo (metros)
%   hdg       – [3×1] direcção de voo normalizada
%   scale     – meia-envergadura (metros); controla toda a escala
%   col_body  – cor da fuselagem  [r g b]
%   col_wing  – cor das asas      [r g b]
%   show_halo – true/false para halo atmosférico (modos de câmara próximos)

    %% ── Eixos do frame de voo ────────────────────────────────────────
    fwd = hdg(:) / norm(hdg);
    rgt = cross(fwd, [0;0;1]);
    if norm(rgt) < 1e-6
        rgt = cross(fwd, [1;0;0]);
    end
    rgt = rgt / norm(rgt);
    up  = cross(rgt, fwd);
    up  = up  / norm(up);

    % Helper: coordenadas locais (fwd, rgt, up) → ponto mundo como linha [x y z]
    P = @(f,r,u) (pos(:) + f*fwd + r*rgt + u*up)';

    %% ── Proporções relativas a scale (meia-envergadura) ─────────────
    Lf   = scale * 0.62;   % comprimento fuselagem frente (do centro)
    Lb   = scale * 0.68;   % comprimento fuselagem trás
    Rf   = scale * 0.09;   % raio da fuselagem
    Rn   = scale * 0.035;  % raio da ponta do nariz

    Ws   = scale * 1.00;   % meia-envergadura asa
    WcR  = scale * 0.36;   % corda asa na raiz
    WcT  = scale * 0.17;   % corda asa na ponta
    Wsw  = scale * 0.20;   % sweep (recuo) da asa

    Ts   = scale * 0.37;   % meia-envergadura estabilizador horiz.
    Tc   = scale * 0.19;   % corda estabilizador
    Tsw  = scale * 0.06;   % sweep do estabilizador

    Fh   = scale * 0.30;   % altura da deriva vertical
    Fc   = scale * 0.22;   % corda da deriva

    tail = -Lb * 0.80;     % posição fwd do centro da cauda

    %% ── FUSELAGEM (elipsoide) ────────────────────────────────────────
    Nu = 24; Nv = 16;
    [Su, Sv] = meshgrid(linspace(0, 2*pi, Nu), linspace(0, pi, Nv));
    cost = cos(Sv);
    % Assimetria frente/trás: nariz mais afilado
    Ex = zeros(size(cost));
    Ex(cost >= 0) =  Lf .* cost(cost >= 0);
    Ex(cost <  0) =  Lb .* cost(cost <  0);
    Ey = Rf * sin(Sv) .* cos(Su);
    Ez = Rf * sin(Sv) .* sin(Su);

    Wx = pos(1) + Ex*fwd(1) + Ey*rgt(1) + Ez*up(1);
    Wy = pos(2) + Ex*fwd(2) + Ey*rgt(2) + Ez*up(2);
    Wz = pos(3) + Ex*fwd(3) + Ey*rgt(3) + Ez*up(3);
    surf(ax, Wx, Wy, Wz, ...
         'FaceColor', col_body, 'EdgeColor', 'none', 'FaceAlpha', 1.0, ...
         'AmbientStrength', 0.28, 'DiffuseStrength', 0.82, ...
         'SpecularStrength', 0.95, 'SpecularExponent', 60);

    % Ponto luminoso no nariz
    scatter3(ax, pos(1)+Lf*fwd(1), pos(2)+Lf*fwd(2), pos(3)+Lf*fwd(3), ...
             12, [1.0 0.95 0.60], 'o', 'filled');

    %% ── ASAS PRINCIPAIS (swept, tapered) ────────────────────────────
    % Cada asa: 4 vértices [raiz-LE, raiz-TE, ponta-TE, ponta-LE]
    for side = [+1, -1]
        w = [ P( WcR*0.38,        side*0.02,  0);   % raiz  LE
              P(-WcR*0.62,        side*0.02,  0);   % raiz  TE
              P(-WcT*0.62-Wsw,    side*Ws,    0);   % ponta TE
              P( WcT*0.38-Wsw,    side*Ws,    0) ]; % ponta LE
        patch(ax, 'XData', w(:,1), 'YData', w(:,2), 'ZData', w(:,3), ...
              'FaceColor', col_wing, 'EdgeColor', 'none', 'FaceAlpha', 1.0, ...
              'AmbientStrength', 0.32, 'DiffuseStrength', 0.80, ...
              'SpecularStrength', 0.45, 'SpecularExponent', 18);
        % Nervura da ponta (winglet subtil)
        wt = [ P( WcT*0.38-Wsw,   side*Ws,   0);
               P(-WcT*0.62-Wsw,   side*Ws,   0);
               P(-WcT*0.42-Wsw,   side*Ws,   Rf*0.8);
               P( WcT*0.28-Wsw,   side*Ws,   Rf*0.8) ];
        patch(ax, 'XData', wt(:,1), 'YData', wt(:,2), 'ZData', wt(:,3), ...
              'FaceColor', col_body, 'EdgeColor', 'none', 'FaceAlpha', 0.85, ...
              'AmbientStrength', 0.32, 'DiffuseStrength', 0.75);
    end

    %% ── ESTABILIZADOR HORIZONTAL ─────────────────────────────────────
    for side = [+1, -1]
        s = [ P(tail + Tc*0.40,       side*0.01,  0);
              P(tail - Tc*0.60,       side*0.01,  0);
              P(tail - Tc*0.60-Tsw,   side*Ts,    0);
              P(tail + Tc*0.40-Tsw,   side*Ts,    0) ];
        patch(ax, 'XData', s(:,1), 'YData', s(:,2), 'ZData', s(:,3), ...
              'FaceColor', col_body, 'EdgeColor', 'none', 'FaceAlpha', 1.0, ...
              'AmbientStrength', 0.30, 'DiffuseStrength', 0.75);
    end

    %% ── DERIVA VERTICAL ─────────────────────────────────────────────
    fin = [ P(tail + Fc*0.50,   0,   0);
            P(tail - Fc*0.50,   0,   0);
            P(tail - Fc*0.15,   0,   Fh);
            P(tail + Fc*0.65,   0,   Fh*0.42) ];
    patch(ax, 'XData', fin(:,1), 'YData', fin(:,2), 'ZData', fin(:,3), ...
          'FaceColor', min(1.0, col_body*1.18 + 0.04), 'EdgeColor', 'none', ...
          'FaceAlpha', 1.0, 'AmbientStrength', 0.30, 'DiffuseStrength', 0.75);

    %% ── HALO ATMOSFÉRICO (modos próximos) ───────────────────────────
    if show_halo
        [Sh_x, Sh_y, Sh_z] = sphere(16);
        R_h = scale * 2.0;
        surf(ax, Sh_x*R_h+pos(1), Sh_y*R_h+pos(2), Sh_z*R_h+pos(3), ...
             'FaceColor', [1.0 0.85 0.20], 'EdgeColor', 'none', ...
             'FaceAlpha', 0.016, 'BackFaceLighting', 'unlit');
    end
end


function cmap = coverage_colormap()
    stops = [0.00 0.04 0.18;
             0.00 0.18 0.52;
             0.00 0.52 0.88;
             0.00 0.82 0.88;
             0.22 0.95 0.65;
             0.80 1.00 0.18;
             1.00 0.95 0.00;
             1.00 1.00 1.00];
    xi   = linspace(0,1,size(stops,1));
    xo   = linspace(0,1,256);
    cmap = max(0,min(1,interp1(xi,stops,xo)));
end


function hex_grid = make_hex_grid(radius, spacing)
    hex_grid = struct('x',{},'y',{});
    n = ceil(radius/spacing)+1; idx = 0;
    for row = -n:n
        for col = -n:n
            cx = col*spacing*1.732;
            cy = row*spacing + mod(col,2)*spacing*0.5;
            if sqrt(cx^2+cy^2) > radius*1.05; continue; end
            idx = idx+1;
            th = linspace(0,2*pi,7);
            hex_grid(idx).x = cx + spacing*0.46*cos(th);
            hex_grid(idx).y = cy + spacing*0.46*sin(th);
        end
    end
end


function alpha = compute_fade(k, Ntime, F)
    if k <= F;              alpha = (k-1)/F;
    elseif k >= Ntime-F+1; alpha = (Ntime-k)/F;
    else;                   alpha = 1.0; end
    alpha = max(0,min(1,alpha));
end


function draw_beam_cone(ax, apex, direction, len, half_deg, alpha, idx)
    N  = 36;
    d  = direction/norm(direction);
    if abs(d(3))<0.9; p1=cross(d,[0;0;1]); else; p1=cross(d,[1;0;0]); end
    p1 = p1/norm(p1); p2 = cross(d,p1);
    r  = len*tand(half_deg);
    t  = linspace(0,2*pi,N);
    base = apex(:) + len*d + r*(cos(t).*p1 + sin(t).*p2);
    Xc = [repmat(apex(1),1,N); base(1,:)];
    Yc = [repmat(apex(2),1,N); base(2,:)];
    Zc = [repmat(apex(3),1,N); base(3,:)];
    cols = [0.18 0.62 1.00;
            0.22 1.00 0.52;
            1.00 0.62 0.12;
            1.00 0.22 0.52;
            0.62 0.32 1.00];
    c = cols(mod(idx-1,size(cols,1))+1,:);
    surf(ax,Xc,Yc,Zc,'FaceColor',c,'EdgeColor','none', ...
         'FaceAlpha',alpha,'BackFaceLighting','unlit');
    plot3(ax,base(1,:),base(2,:),base(3,:),'Color',[c 0.42],'LineWidth',0.9);
end


function R = attitude_to_rotation(att)
    r = deg2rad(att.roll_deg);
    p = deg2rad(att.pitch_deg);
    y = deg2rad(att.yaw_deg);
    R = [cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1] * ...
        [cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)] * ...
        [1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
end


function [G_dBi, G_lin] = antenna_gain_local(arrays_cfg, drone_attitude, az_deg, el_deg)
    R_b2w = attitude_to_rotation(drone_attitude);
    az_r  = deg2rad(az_deg); el_r = deg2rad(el_deg);
    u_w   = [sin(az_r)*cos(el_r); cos(az_r)*cos(el_r); sin(el_r)];
    G_all = zeros(1,numel(arrays_cfg));
    for ii = 1:numel(arrays_cfg)
        ac    = arrays_cfg(ii);
        az_bs = deg2rad(ac.boresight_az); el_bs = deg2rad(ac.boresight_el);
        bs    = R_b2w * [cos(el_bs)*sin(az_bs); -cos(el_bs)*cos(az_bs); sin(el_bs)];
        bs    = bs/norm(bs);
        cp    = max(-1,min(1,dot(bs,u_w)));
        if strcmpi(ac.type,'preset'); G_all(ii) = gain_preset_local(ac,cp);
        else; G_all(ii) = 1; end
    end
    G_lin = max(G_all); G_dBi = 10*log10(max(G_lin,1e-10));
end


function G_lin = gain_preset_local(ac, cos_psi)
    G0 = 10^(ac.G_elem_dBi/10);
    switch lower(ac.preset)
        case 'omni';   G_lin = G0;
        case 'dipole'; G_lin = 1.64*G0*max(0,1-cos_psi^2);
        case 'patch'
            if cos_psi<=0; G_lin=0; else; G_lin=G0*cos_psi; end
        case 'yagi'
            n = log(0.5)/log(cosd(30));
            if isfield(ac,'HPBW_half_deg')&&ac.HPBW_half_deg>0
                n = log(0.5)/log(cosd(ac.HPBW_half_deg)); end
            if cos_psi<=0; G_lin=0; else; G_lin=G0*cos_psi^n; end
        case 'array'
            if cos_psi<=0; G_lin=0; return; end
            n = log(0.5)/log(cosd(65));
            if isfield(ac,'HPBW_half_deg')&&ac.HPBW_half_deg>0
                n = log(0.5)/log(cosd(ac.HPBW_half_deg)); end
            N_t = ac.Nx*ac.Ny; e = 1.0;
            if isfield(ac,'eta_element'); e=e*ac.eta_element; end
            if isfield(ac,'eta_array');   e=e*ac.eta_array; end
            G_lin = (cos_psi^n)*G0*N_t*e;
        otherwise; G_lin = G0;
    end
end