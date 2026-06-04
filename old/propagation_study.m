clc; clear; close all;

%% Free Space Path Loss

c = physconst('lightspeed');

freq = [2.4e9 5e9 6e9 3.9e9];   % Hz
lambda = c ./ freq;             % wavelength (m)

R0 = linspace(1, 10e3, 1000);   % distance in meters                   

apathloss = fspl(R0.', c ./ freq);   % FSPL (dB)

figure;
plot(R0/1e3, apathloss, 'LineWidth', 1.5);
grid on;

ylim([70 150]);

legend('WiFi6/7 (2.4GHz)','WiFi6/7 (5GHz)','WiFi7 (6GHz)','5G', ...
       'Location','best','FontSize',14);

set(gca,'FontSize',16);
xlabel('Range (km)','FontSize',16);
ylabel('Path Loss (dB)','FontSize',16);
title('Free Space Path Loss vs Distance','FontSize',18);

%% Rain attenuation vs frequency with signal markers and vertical lines

R0 = 1e3;                % 1 km range
rainrate = [1 4 20];     % mm/h
el = 0;
tau = 0;
freq = (1:7).' * 1e9;  

sig_freq = [2.4e9 5e9 6e9 3.9e9]; 
sig_names = {'WiFi6/7 (2.4GHz)','WiFi6/7 (5GHz)','WiFi7 (6GHz)','5G'};

% Rain attenuation calculation
for m = 1:numel(rainrate)
    rainloss_itu(:,m) = rainpl(R0,freq,rainrate(m),el,tau)';
end

% Plot
figure; hold on;
colors = lines(numel(rainrate));
for m = 1:numel(rainrate)
    plot(freq/1e9,rainloss_itu(:,m),'Color',colors(m,:),'LineWidth',1.5);
end

for s = 1:numel(sig_freq)
    xline(sig_freq(s)/1e9,'--k','LineWidth',1);
    [~,idx] = min(abs(freq - sig_freq(s)));  
    text(sig_freq(s)/1e9, max(rainloss_itu(idx,:))+0.03, sig_names{s}, ...
        'HorizontalAlignment','center','FontWeight','bold','FontSize',16);
end

grid on;
set(gca,'FontSize',16);       % Axes
xlabel('Frequency (GHz)','FontSize',16);
ylabel('Attenuation at 1 km (dB)','FontSize',16);
title('Rain Attenuation for Horizontal Polarization','FontSize',18);

lgd = legend('Light Rain (1 mm/h)','Moderate Rain (4 mm/h)','Heavy Rain (20 mm/h)', ...
             'Location','best');
lgd.FontSize = 14;






