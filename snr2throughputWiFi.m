function TeffMbps = snr2throughputWiFi(snr_dB)

% reference: https://www.candelatech.com/courses-2023/Session2c_notes.pdf

% it was considered Wi-Fi 6, 20MHz channels, and 800 ns of guard interval
%% WIFI PHY CONFIG (IEEE 802.11ax)
wifi.BW_Hz = 20e6;        % 20 MHz bandwidth
wifi.Ntx   = 1;          % 1 TX antenna
wifi.Nrx   = 1;          % 1 RX antenna
wifi.NSS   = 1;          % 1x1 MIMO => 1 spatial stream
wifi.NSD   = 234;        % Number of data subcarriers per resource unit
wifi.Tdft  = 12.8e-6;    % OFDM Symbol Duration


%% MCS Spectral Efficiency (bits/s/Hz)
GI = 800e-9; % guard interval

etaTable = [
    8.6;    % MCS 0  BPSK 1/2
    17.2;    % MCS 1  QPSK 1/2
    25.8;    % MCS 2  QPSK 3/4
    34.4;    % MCS 3  16-QAM 1/2
    51.6;    % MCS 4  16-QAM 3/4
    68.8;    % MCS 5  64-QAM 2/3
    77.4;    % MCS 6  64-QAM 3/4
    86;    % MCS 7  64-QAM 5/6
    103.2;    % MCS 8  256-QAM 3/4
    114.7;   % MCS 9  256-QAM 5/6
    129;    % MCS 10 1024-QAM 3/4
    143.4    % MCS 11 1024-QAM 5/6  
];
RTable = [
    1/2;   % MCS 0  BPSK 1/2
    1/2;   % MCS 1  QPSK 1/2
    3/4;   % MCS 2  QPSK 3/4
    1/2;   % MCS 3  16-QAM 1/2
    3/4;   % MCS 4  16-QAM 3/4
    2/3;   % MCS 5  64-QAM 2/3
    3/4;   % MCS 6  64-QAM 3/4
    5/6;   % MCS 7  64-QAM 5/6
    3/4;   % MCS 8  256-QAM 3/4
    5/6;   % MCS 9  256-QAM 5/6
    3/4;   % MCS 10 1024-QAM 3/4
    5/6    % MCS 11 1024-QAM 5/6
];


%% === BLER model (link abstraction) ===
a = 5;   

TeffMbps = zeros(numel(snr_dB),1);

for i = 1:numel(snr_dB)

    snrLin = 10^(snr_dB(i)/10);

    % Shannon capacity
    C = log2(1 + snrLin);

    bestThroughput = 0;

    for k = 1:numel(etaTable)

        eta = etaTable(k);
        R = RTable(k);
        delta = C - eta;

        % BLER based on the gap to Shannon
        bler = 1 / (1 + exp(a * delta));

        % Effective throughput (1x1 MIMO)
        T = (wifi.NSD * eta * R * wifi.NSS/(GI + wifi.Tdft)) * (1 - bler);

        bestThroughput = max(bestThroughput, T);
    end

    TeffMbps(i) = bestThroughput / 1e6;
end
end
