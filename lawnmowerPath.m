function [x,y] = lawnmowerPath(cfg, s)

    L = cfg.length_m;
    W = cfg.width_m;
    nLines = cfg.nLines;

    lineLength = L;
    lineSpacing = W/(nLines-1);

    x = zeros(size(s));
    y = zeros(size(s));

    dLine = 2*L; % ida e volta
    lineIdx = floor(s / dLine) + 1;
    lineIdx = min(lineIdx, nLines);

    dLocal = mod(s, dLine);

    for i = 1:numel(s)
        if mod(lineIdx(i),2)==1
            x(i) = -L/2 + min(dLocal(i), L);
        else
            x(i) =  L/2 - min(dLocal(i), L);
        end
        y(i) = -W/2 + (lineIdx(i)-1)*lineSpacing;
    end
end
