function convert_tif_to_dted(tif_file, dt1_file)

    [Z_orig, R] = readgeoraster(tif_file, 'OutputType','double');
    if ndims(Z_orig) == 3; Z_orig = Z_orig(:,:,1); end
    Z_orig(~isfinite(Z_orig)) = 0;

    lat_lim = R.LatitudeLimits;
    lon_lim = R.LongitudeLimits;
    lat0 = floor(lat_lim(1));
    lon0 = floor(lon_lim(1));

    nLat = 1201; nLon = 1201;

    fprintf('  Célula DTED1: Lat [%d,%d]  Lon [%d,%d]\n', lat0,lat0+1,lon0,lon0+1);

    lat_dted = lat0 + (0:nLat-1)' / (nLat-1);
    lon_dted = lon0 + (0:nLon-1)  / (nLon-1);

    [nR,nC] = size(Z_orig);
    lat_orig = linspace(lat_lim(1),lat_lim(2),nR);
    lon_orig = linspace(lon_lim(1),lon_lim(2),nC);
    F = griddedInterpolant({lat_orig,lon_orig},flipud(Z_orig),'linear','nearest');
    [LON_g,LAT_g] = meshgrid(lon_dted,lat_dted);
    Z_dted = int16(round(F(LAT_g,LON_g)));
    outside = LAT_g<lat_lim(1)|LAT_g>lat_lim(2)|...
              LON_g<lon_lim(1)|LON_g>lon_lim(2);
    Z_dted(outside) = 0;

    expected = 80 + 648 + 2700 + nLon*(8 + nLat*2 + 4);

    lat_abs = abs(lat0); lon_abs = abs(lon0);
    lat_h = 'N'; if lat0 < 0; lat_h = 'S'; end
    lon_h = 'E'; if lon0 < 0; lon_h = 'W'; end

    ref_file = 'C:\Program Files\MATLAB\R2024b\toolbox\shared\terrain\n39_w106_3arc_v2.dt1';
    fid_ref  = fopen(ref_file,'rb','ieee-be');
    raw_ref  = fread(fid_ref, 80+648+2700, 'uint8')';
    fclose(fid_ref);

    fid = fopen(dt1_file,'wb','ieee-be');
    if fid==-1; error('Não consegue criar: %s', dt1_file); end

    %% UHL
    uhl = raw_ref(1:80);
    uhl(5:12)  = uint8(sprintf('%03d0000%s', lon_abs, lon_h));
    uhl(13:20) = uint8(sprintf('%03d0000%s', lat_abs, lat_h));
    uhl(49:52) = uint8(sprintf('%04d', nLon));
    uhl(53:56) = uint8(sprintf('%04d', nLat));
    fwrite(fid, uhl, 'uint8');

    %% DSI
    dsi = raw_ref(81:728);

    % Coords: 4 pares (lat7+lon8)=15 chars, total 60 chars
    % Terminam em DSI[264] → começam em DSI[205]
    mk7 = @(d,h) sprintf('%02d0000%s', abs(d), h);   % 7 chars
    mk8 = @(d,h) sprintf('%03d0000%s', abs(d), h);   % 8 chars

    coords = [mk7(lat0,  lat_h) mk8(lon0,  lon_h) ...  % SW
              mk7(lat0+1,lat_h) mk8(lon0,  lon_h) ...  % NW
              mk7(lat0+1,lat_h) mk8(lon0+1,lon_h) ...  % NE
              mk7(lat0,  lat_h) mk8(lon0+1,lon_h)];    % SE
    assert(numel(coords)==60, 'coords deve ter 60 chars, tem %d', numel(coords));

    dsi(205:264) = uint8(coords);

    % Verificar que campos críticos [274:291] ficaram intactos
    fprintf('  DSI[274:291] após patch = "%s"\n', char(dsi(274:291)));

    fwrite(fid, dsi, 'uint8');

    %% ACC
    acc = raw_ref(729:3428);
    fwrite(fid, acc, 'uint8');

    %% Data Records
    for c = 1:nLon
        col = Z_dted(:,c);
        idx = c-1;
        d5=floor(idx/100000); r=mod(idx,100000);
        d4=floor(r/10000);    r=mod(r,10000);
        d3=floor(r/1000);     r=mod(r,1000);
        d2=floor(r/100);      r=mod(r,100);
        d1=floor(r/10);       d0=mod(r,10);
        rec_hdr = uint8([0xAA, d5*16+d4, d3*16+d2, d1*16+d0, ...
            floor(idx/256), mod(idx,256), ...
            floor(nLat/256), mod(nLat,256)]);
        fwrite(fid, rec_hdr, 'uint8');
        fwrite(fid, col, 'int16');
        col_bytes = typecast(col,'uint8');
        checksum  = uint32(sum(uint32(rec_hdr))) + ...
                    uint32(sum(uint32(col_bytes)));
        fwrite(fid, checksum, 'uint32');
    end

    fclose(fid);

    info = dir(dt1_file);
    ok   = (info.bytes == expected);
    fprintf('  %d bytes | esperado %d | %s\n', info.bytes, expected, ...
            sel(ok,'TAMANHO OK ✓','TAMANHO ERRADO ✗'));
end

function r = sel(cond,a,b); if cond; r=a; else; r=b; end; end