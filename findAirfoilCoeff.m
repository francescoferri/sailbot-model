function [cl, cd, cdp, cm] = findAirfoilCoeff(reynolds_number, alpha, M)

% Find Closest Re that is tabulated
reynolds_tabulated=transpose(unique(M(:,1))); % Values of Re Tabulated, 500-200-100k
[~,idx]=min(abs(reynolds_tabulated-reynolds_number));
ReTab=reynolds_tabulated(idx); % Closest Re tabulated in M

% Find Closest alpha within closest Re
alphas=zeros;
for i = 1:size(M,1)
    if (M(i,1)==ReTab)
        alphas(i,1) = M(i,2); % get alphas
    end 
end

[~,idx]=min(abs(alphas-alpha));
alphaTab=alphas(idx); % Closest alpha tabulated in M

% Parsing answer
for i = 1:size(M,1)
    if (M(i,1)==ReTab && M(i,2)==alphaTab)
        cl=M(idx, 3);
        cd=M(idx, 4);
        cdp=M(idx, 5);
        cm=M(idx, 6);
    end
end

end

