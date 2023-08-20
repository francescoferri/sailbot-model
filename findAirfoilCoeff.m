function [cl, cd, cdp, cm, alpha_v, at_stall] = findAirfoilCoeff(reynolds_number, alpha, airfoil_matrix)

% Find Closest Re that is tabulated
reynolds_tabulated = transpose(unique(airfoil_matrix(:,1))); % Values of Re Tabulated, 500-200-100k
[~,idx] = min(abs(reynolds_tabulated-reynolds_number)); % find index at which difference is smallest
ReTab = reynolds_tabulated(idx); % parse closest Re tabulated in M

at_stall = false;

% Find alphas available for closest reynolds_number tabulated
data = [];
k=0;
for i = 1:size(airfoil_matrix,1)
    if (airfoil_matrix(i,1) == ReTab)

        k = k + 1;
        data(k,:) = airfoil_matrix(i,2:7); % extract data
        
    end 
end

alphas = data(:,1);

alpha_max = max(alphas);
alpha_min = min(alphas);

if (alpha > alpha_max)
    at_stall = true;
    alpha = alpha_max;
elseif (alpha < alpha_min)
    at_stall = true;
    alpha = alpha_min;
end

idx = find(alphas(:,1) == alpha);

if (idx) %there is an exact match

    % parsing answer
    cl = data(idx, 2);
    cd = data(idx, 3);
    cdp = data(idx, 4);
    cm = data(idx, 5);

else % no exact match is found, interpolate

    alpha_differences_sorted = sort(abs(alpha-alphas));
    lower_idx = find(abs(alpha-alphas)==alpha_differences_sorted(1));
    upper_idx = find(abs(alpha-alphas)==alpha_differences_sorted(2));

    lower_value = alphas(lower_idx);
    upper_value = alphas(upper_idx);
    alpha_v = [lower_value upper_value];

    cl = interp1(alpha_v, [data(lower_idx,2) data(upper_idx, 2)], alpha, 'linear');
    cd = interp1(alpha_v, [data(lower_idx,2) data(upper_idx, 3)], alpha, 'linear');
    cdp = interp1(alpha_v, [data(lower_idx,2) data(upper_idx, 4)], alpha, 'linear');
    cm = interp1(alpha_v, [data(lower_idx,2) data(upper_idx, 5)], alpha, 'linear');

end

end

