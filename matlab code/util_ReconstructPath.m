% reconstruction the path
function [index_list] = util_ReconstructPath(closelist)
% the last point of the closed list should be the destination
index_list = [closelist(end,1) closelist(end,2)];
parent_index = [closelist(end,6) closelist(end,7)];
while(~util_CompareLocation(parent_index(1), parent_index(2), -1, -1))
    index_list = [parent_index(1), parent_index(2);
                  index_list];
    % move to the parent node
    [~,location] = util_IsLocationinList(closelist, parent_index(1), parent_index(2));
    parent_index(1) = closelist(location,6);
    parent_index(2) = closelist(location,7);
end

