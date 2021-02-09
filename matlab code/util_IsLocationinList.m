% determine whether a point is in the open or closed list
% in true, not in false
function [isin,location]= util_IsLocationinList(list, index_x, index_y)
% 1. index_x, 2. index_y, 3. g_cost, 4. h_cost, 5. f_cost, 6. parent index_x
L1 = list(:,1) - index_x*ones(size(list(:,1)));
% if there are 0 in L1 and L2, and the corresponding indexes are the same,
% then return
R1 = find(~L1);
dimR1 = size(R1);
location = [];
isin = false;
if dimR1(1) ==0 % mean no same points
    isin = false;
else
    for i = 1 : dimR1(1) % if there are zero entries in the difference, check 
        % whether corresponding index y is same
        if ((list(R1(i),2)-index_y) ==0)
               isin = true; % flag is true if at least one has been found
               location = [location;
                            R1(i)];
        end
    end
end

