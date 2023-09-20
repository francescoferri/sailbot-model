function [cl, cd] = findCdClwithRatio(reynolds_number, cl_over_cd, airfoil_matrix)

% Find Closest Re that is tabulated
reynolds_tabulated = transpose(unique(airfoil_matrix(:,1))); % Values of Re Tabulated, 500-200-100k
[~,idx] = min(abs(reynolds_tabulated-reynolds_number)); % find index at which difference is smallest
ReTab = reynolds_tabulated(idx); % parse closest Re tabulated in M

% Extract data for given raynolds number
data = [];
k=0;
for i = 1:size(airfoil_matrix,1)
    if (airfoil_matrix(i,1) == ReTab)

        k = k + 1;
        data(k,:) = airfoil_matrix(i,2:7); % extract data
        
    end 
end

cls = data(:,2);
cds = data(:,3);
cl_over_cd_s = cls./cds;

cl_over_cd_max = max(cl_over_cd_s);
cl_over_cd_min = min(cl_over_cd_s);

if (cl_over_cd > cl_over_cd_max)
    error "cl_over_cd out of tabulated values"
elseif (cl_over_cd < cl_over_cd_min)
    error "cl_over_cd out of tabulated values"
end

idx = find(cl_over_cd_s(:,1) == cl_over_cd);

if (idx) %there is an exact match

    % parsing answer
    cl = data(idx, 2);
    cd = data(idx, 3);

else % no exact match is found, interpolate

    cl_over_cd_differences_sorted = sort(abs(cl_over_cd-cl_over_cd_s));
    lower_idx = find(abs(cl_over_cd-cl_over_cd_s) == cl_over_cd_differences_sorted(1));
    upper_idx = find(abs(cl_over_cd-cl_over_cd_s) == cl_over_cd_differences_sorted(2));

    lower_value = cl_over_cd_s(lower_idx);
    upper_value = cl_over_cd_s(upper_idx);
    cl_over_cd_s_v = [lower_value upper_value];

    cl = interp1(cl_over_cd_s_v, [data(lower_idx,2) data(upper_idx, 2)], cl_over_cd, 'linear');
    cd = interp1(cl_over_cd_s_v, [data(lower_idx,3) data(upper_idx, 3)], cl_over_cd, 'linear');

end

end

