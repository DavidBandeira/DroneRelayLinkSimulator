clc; clear; close all;


%%  CONFIGURAÇÃO

reduzir_visual = true;
fator          = 5;          % Factor de subamostragem para visualização/render
pasta          = 'pedrogao';
output_name    = 'pedrogao_terrain';   % Base para os ficheiros .mat e .tif de saída


%%  DESCOBERTA E PARSING DOS FICHEIROS


files      = dir(fullfile(pasta, 'MDT-2m-*.tif'));
file_names = fullfile({files.folder}, {files.name});
n_files    = length(file_names);

if n_files == 0
    error('Nenhum ficheiro MDT-2m-*.tif encontrado em "%s".', pasta);
end

coords = zeros(n_files, 2);   % [lon_codigo, lat_codigo]

for k = 1:n_files
    [~, name, ~] = fileparts(file_names{k});
    tokens = regexp(name, 'MDT-2m-(\d{6})-\d{2}-\d{4}_v01', 'tokens');
    if isempty(tokens)
        error('Ficheiro "%s" não segue o padrão esperado.', name);
    end
    coord_str    = tokens{1}{1};
    coords(k, 1) = str2double(coord_str(1:3));   % código X (pseudo-lon)
    coords(k, 2) = str2double(coord_str(4:6));   % código Y (pseudo-lat)
end


%%  CARREGAMENTO E MONTAGEM DOS TILES


unique_lon = sort(unique(coords(:, 1)));
unique_lat = sort(unique(coords(:, 2)));

Z_all = cell(length(unique_lat), length(unique_lon));
R_all = cell(length(unique_lat), length(unique_lon));

for k = 1:n_files
    lon_c  = coords(k, 1);
    lat_c  = coords(k, 2);
    i_lat  = find(unique_lat == lat_c);
    j_lon  = find(unique_lon == lon_c);

    [Z, R] = readgeoraster(file_names{k});
    Z      = double(Z);
    Z(Z == -9999) = NaN;

    % Garantir orientação Norte-para-baixo (linha 1 = Norte)
    if R.YWorldLimits(1) > R.YWorldLimits(2)
        Z = flipud(Z);
    end

    Z_all{i_lat, j_lon} = Z;
    R_all{i_lat, j_lon} = R;
end


%%  CONCATENAÇÃO: linhas = latitude (Sul→Norte), colunas = longitude


% Determinar tamanho de tile de referência (para preencher lacunas)
sz_ref = [];
for i = 1:numel(Z_all)
    if ~isempty(Z_all{i}); sz_ref = size(Z_all{i}); break; end
end

rows_concat = cell(length(unique_lat), 1);
for i_lat = 1:length(unique_lat)
    row_tiles = Z_all(i_lat, :);
    for j_lon = 1:length(unique_lon)
        if isempty(row_tiles{j_lon})
            row_tiles{j_lon} = NaN(sz_ref);   % preencher tile em falta
        end
    end
    rows_concat{i_lat} = horzcat(row_tiles{:});
end

% Inverter ordem das latitudes: índice 1 passa a ser Norte
rows_concat = flipud(rows_concat);
Z_total     = vertcat(rows_concat{:});


%%  GEOREFERENCIAÇÃO EM LAT/LON (graus)


% --- Encontrar R_ref válido e limites GLOBAIS do mosaico --------------
R_ref        = [];
x_min_global =  Inf;
x_max_global = -Inf;
y_min_global =  Inf;
y_max_global = -Inf;

for i = 1:numel(R_all)
    if isempty(R_all{i}), continue; end
    R_k = R_all{i};
    if isempty(R_ref), R_ref = R_k; end   % guardar o primeiro para CRS/resolução

    x_min_global = min(x_min_global, min(R_k.XWorldLimits));
    x_max_global = max(x_max_global, max(R_k.XWorldLimits));
    y_min_global = min(y_min_global, min(R_k.YWorldLimits));
    y_max_global = max(y_max_global, max(R_k.YWorldLimits));
end

[n_rows, n_cols] = size(Z_total);

% Vectores de coordenadas no CRS original com base nos limites GLOBAIS
% (Norte → Sul para corresponder à orientação de Z_total: linha 1 = Norte)
x_vec = x_min_global + (0:n_cols-1) * R_ref.CellExtentInWorldX;
y_vec = y_max_global - (0:n_rows-1) * abs(R_ref.CellExtentInWorldY);

[Xcrs, Ycrs] = meshgrid(x_vec, y_vec);

% Converter para Latitude/Longitude WGS84
if isprop(R_ref, 'ProjectedCRS') && ~isempty(R_ref.ProjectedCRS)
    [lat_full, lon_full] = projinv(R_ref.ProjectedCRS, Xcrs, Ycrs);
    fprintf('CRS detectado: %s → convertido para WGS84 lat/lon.\n', ...
            R_ref.ProjectedCRS.Name);
else
    lon_full = Xcrs;
    lat_full = Ycrs;
    fprintf('Sem CRS projectado detectado — assumindo coordenadas já em graus.\n');
end

% Diagnóstico: confirmar cantos do mosaico
fprintf('Canto NW: lat=%.6f  lon=%.6f\n', lat_full(1,1),     lon_full(1,1));
fprintf('Canto NE: lat=%.6f  lon=%.6f\n', lat_full(1,end),   lon_full(1,end));
fprintf('Canto SW: lat=%.6f  lon=%.6f\n', lat_full(end,1),   lon_full(end,1));
fprintf('Canto SE: lat=%.6f  lon=%.6f\n', lat_full(end,end), lon_full(end,end));

%%  EXPORTAÇÃO  –  versão completa e versão leve


% --- Versão completa (lat/lon em graus, resolução original) -----------
X_full = lon_full;   % longitude (graus)
Y_full = lat_full;   % latitude  (graus)
Z_full = Z_total;    % altitude  (metros)

save(sprintf('%s_full.mat', output_name), 'Z_full', 'X_full', 'Y_full');
fprintf('Guardado: %s_full.mat  [%d × %d]\n', output_name, n_rows, n_cols);

% --- Versão leve (subamostragem para renderer e Siteviewer) -----------
if reduzir_visual
    idx_r   = 1:fator:n_rows;
    idx_c   = 1:fator:n_cols;
    X_light = lon_full(idx_r, idx_c);   % longitude
    Y_light = lat_full(idx_r, idx_c);   % latitude
    Z_light = Z_total(idx_r, idx_c);    % altitude

    save(sprintf('%s_light.mat', output_name), 'Z_light', 'X_light', 'Y_light');
    fprintf('Guardado: %s_light.mat [%d × %d]\n', output_name, ...
            length(idx_r), length(idx_c));
end


%%  EXPORTAÇÃO GEOTIFF PARA addCustomTerrain() DO SITEVIEWER
try
    lat_min = min(lat_full(:));  lat_max = max(lat_full(:));
    lon_min = min(lon_full(:));  lon_max = max(lon_full(:));

    R_wgs84 = georasterref( ...
        'LatitudeLimits',  [lat_min, lat_max], ...
        'LongitudeLimits', [lon_min, lon_max], ...
        'RasterSize',      size(Z_total), ...
        'ColumnsStartFrom','south');   % linha 1 = Sul → Norte crescente

    % Z deve estar orientado Sul→Norte para ColumnsStartFrom='south'
    Z_export = flipud(Z_total);
    Z_export(isnan(Z_export)) = -9999;

    tif_name = sprintf('%s_wgs84.tif', output_name);
    geotiffwrite(tif_name, Z_export, R_wgs84, 'CoordRefSysCode', 4326);
    fprintf('GeoTIFF exportado: %s\n', tif_name);
    fprintf('\nPara usar no Siteviewer:\n');
    fprintf('  addCustomTerrain(''%s'', ''%s'')\n', output_name, tif_name);
    fprintf('  txsite(..., ''Terrain'', ''%s'')\n', output_name);
    fprintf('  rxsite(..., ''Terrain'', ''%s'')\n\n', output_name);
catch ME
    warning('geotiffwrite falhou (%s). O .mat foi guardado na mesma.', ME.message);
end


%%  VISUALIZAÇÃO


if reduzir_visual
    lon_vis = X_light;
    lat_vis = Y_light;
    Z_vis   = Z_light;
else
    lon_vis = lon_full;
    lat_vis = lat_full;
    Z_vis   = Z_total;
end

figure('Name','Terreno – coordenadas geográficas','Color','w');
surf(lon_vis, lat_vis, Z_vis, 'EdgeColor', 'none');
colormap('turbo'); colorbar;
xlabel('Longitude (°)'); ylabel('Latitude (°)'); zlabel('Altitude (m)');
title(sprintf('MDT combinado – %d×%d tiles  (factor visual ×%d)', ...
      length(unique_lon), length(unique_lat), fator));
view(3); axis tight; camlight; lighting gouraud; rotate3d on;