function link = computeLink(txPos, rxPos, cfg, map)
% computeLink
% Generic radio link computation (WiFi, 5G, etc.)
% Evaluates received power, SNR, theoretical capacity, and link status
%
% INPUTS:
%   txPos - struct with transmitter position (lat, lon, alt)
%   rxPos - struct with receiver position (lat, lon, alt)
%   cfg   - link configuration struct (frequency, power, antennas, etc.)
%   map   - visualization configuration struct
%
% OUTPUT:
%   link struct containing link metrics

    %% Constants
    % Thermal noise power spectral density at 290 K
    kT_dBm_perHz = -174;

    %% Create TX
    % Convert transmit power from dBm to Watts
    txPower_W = 10.^((cfg.txPower_dBm - 30)/10);

    % Define transmitter site
    tx = txsite( ...
        "Latitude",  txPos.lat, ...            % transmitter latitude
        "Longitude", txPos.lon, ...            % transmitter longitude
        "AntennaHeight", txPos.alt, ...        % transmitter antenna height
        "TransmitterFrequency", cfg.fHz, ...   % operating frequency
        "TransmitterPower", txPower_W);        % transmit power (W)

    %% Create RX
    % Define receiver site
    rx = rxsite( ...
        "Latitude",  rxPos.lat, ...             % receiver latitude
        "Longitude", rxPos.lon, ...             % receiver longitude
        "AntennaHeight", rxPos.alt, ...         % receiver antenna height
        "ReceiverSensitivity", cfg.rxSens_dBm);% receiver sensitivity

    %% Propagation model
    % Select propagation model
    switch lower(cfg.propModel)
        case "freespace"
            % Ideal line-of-sight free-space propagation
            pm = propagationModel("freespace");
        case "longley-rice"
            % Longley–Rice irregular terrain model
            pm = propagationModel("longley-rice");
        otherwise
            error("Unknown propagation model");
    end

    %% Received power
    % Compute received signal strength (dBm)
    Prx_dBm = sigstrength(rx, tx, pm);

    %% Noise
    % Total noise power (dBm)
    % Thermal noise + bandwidth + noise figure
    noise_dBm = kT_dBm_perHz + 10*log10(cfg.BHz) + cfg.NF_dB;

    %% SNR
    % Signal-to-noise ratio
    SNR_dB = Prx_dBm - noise_dBm;

    %% Capacity (Shannon)
    % Convert SNR to linear scale
    SNR_lin = 10.^(SNR_dB/10);

    % Shannon theoretical maximum capacity
    C_Mbps  = cfg.BHz * log2(1 + SNR_lin) / 1e6;

    %% Margin
    % Link margin considering receiver sensitivity and fade margin
    margin_dB = Prx_dBm - (cfg.rxSens_dBm + cfg.fadeMargin_dB);

    %% Outputs
    % Output structure with link performance metrics
    link.Prx_dBm        = Prx_dBm;        % received power
    link.SNR_dB         = SNR_dB;         % SNR
    link.capacity_Mbps = C_Mbps;          % theoretical capacity
    link.margin_dB     = margin_dB;       % link margin
    link.linkOK        = margin_dB >= 0;  % link availability flag

    %% Mapping
    % Create map viewer if any visualization is enabled
    if (map.tx || map.rx || map.Tax)
        viewer = siteviewer("Basemap","satellite");
    end

    % Show transmitter on map
    if (map.tx); show(tx); end

    % Show receiver on map
    if (map.tx); show(rx); end
    
    if (map.Tax)
        % Compute spatial SINR distribution
        pd = sinr(tx, pm, ...
              "MaxRange", map.MapRange, ...     % maximum analysis range
              "Resolution", map.Resolution, ... % spatial resolution
              "ReceiverNoisePower", noise_dBm);

        % Convert SINR to linear scale
        SINR_linear = 10.^(pd.Data.SINR/10);

        % Extract grid coordinates
        lat = pd.Data.Latitude;
        lon = pd.Data.Longitude;

        % Compute theoretical data rate at each map point
        pl = (cfg.BHz * log2(1 + SINR_linear))/1e6;

        % Create table for mapping
        tbl = table(lat, lon, pl, ...
            'VariableNames', {'Latitude','Longitude','DataTax'});

        % Create propagation data object
        pd_datatax = propagationData(tbl, ...
            'DataVariableName', 'DataTax');
        
        % Plot data rate contour map
        contour(pd_datatax, ...
            "Map", viewer, ...
            "Type", "custom", ...
            "ColorLimits", [0 300], ...
            "Colormap", flipud(jet));  
    end

end

