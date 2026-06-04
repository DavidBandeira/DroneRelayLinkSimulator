function TeffMbps = snr2throughput5G(snr_dB)
%% === NR CONFIG ===
carrier = nrCarrierConfig;
carrier.NSizeGrid = 51;              % ~20 MHz @ 30 kHz
carrier.SubcarrierSpacing = 30;
carrier.CyclicPrefix = 'Normal';

pdsch = nrPDSCHConfig;
pdsch.NumLayers = 1;                 
pdsch.PRBSet = 0:carrier.NSizeGrid-1;

%% === 3GPP MCS TABLE (Spectral Efficiency) ===
% TS 38.214 Table 5.1.3.1-2 (subset)
etaTable = [
    0.2344;
    0.3770;
    0.6016;
    0.8770;
    1.1758;
    1.4766;
    1.6953;
    1.9141;
    2.1602;
    2.4063;
    2.5703;
    2.7305;
    3.0293;
    3.3223;
    3.6094;
    3.9023;
    4.2129;
    4.5234;
    4.8164;
    5.1152;
    5.3320;
    5.5547;
    5.8906;
    6.2266;
    6.5703;
    6.9141;
    7.1602;
    7.4063
];

%% === Bandwidth ===
BW = carrier.NSizeGrid * 12 * carrier.SubcarrierSpacing * 1e3; % Hz

%% === BLER logistic steepness ===
a = 6;   % controls how aggressive the BLER transition is

TeffMbps = zeros(numel(snr_dB),1);

for i = 1:numel(snr_dB)

    snrLin = 10^(snr_dB(i)/10);
    C = log2(1 + snrLin);   % Shannon capacity (bits/s/Hz)

    bestThroughput = 0;

    for k = 1:numel(etaTable)

        eta = etaTable(k);
        delta = C - eta;

        % BLER model (link abstraction)
        bler = 1 / (1 + exp(a * delta));

        % Effective throughput
        T = BW * eta * pdsch.NumLayers * (1 - bler);

        bestThroughput = max(bestThroughput, T);
    end

    TeffMbps(i) = bestThroughput / 1e6;
end
end

