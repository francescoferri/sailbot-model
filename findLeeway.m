function [beta, cd, cdp, cm] = findLeeway(reynolds_number, cl, airfoil_matrix)

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

cls = data(:,2);

%side drift should be way less 5 degrees anyway, and if you use max() it
%messes up later in the algorithm because the CL increase then decreases
%from 0 to stall, so interp1 throws and error if alpha is increasing but cl
%is decreasing
cl_max = cls(end); 
cl_min = cls(1);

if (cl >= cl_max)
    at_stall = true;
    cl = cl_max;
elseif (cl <= cl_min)
    at_stall = true;
    cl = cl_min;
end

idx = find(cls(:,1) == cl);

if (idx) %there is an exact match

    % parsing answer
    beta = data(idx, 1);
    cd = data(idx, 3);
    cdp = data(idx, 4);
    cm = data(idx, 5);

else % no exact match is found, interpolate

    cl_differences_sorted = sort(abs(cl-cls));
    lower_idx = find(abs(cl-cls)==cl_differences_sorted(1));
    if(data(lower_idx,2) > cl) 
        upper_idx = lower_idx - 1;
    else
        upper_idx = lower_idx + 1;
    end

    lower_value = cls(lower_idx);
    upper_value = cls(upper_idx);
    cl_v = [lower_value upper_value];

    beta = interp1(cl_v, [data(lower_idx,1) data(upper_idx, 1)], cl, 'linear');
    cd = interp1(cl_v, [data(lower_idx,3) data(upper_idx, 3)], cl, 'linear');
    cdp = interp1(cl_v, [data(lower_idx,4) data(upper_idx, 4)], cl, 'linear');
    cm = interp1(cl_v, [data(lower_idx,5) data(upper_idx, 5)], cl, 'linear');


end

end

