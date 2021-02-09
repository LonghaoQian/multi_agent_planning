% determine of the index of the point in the open list with lowest f cost
% -1 means that the open set is empty
% openlist:  1. index_x, 2. index_y, 3. g_cost, 4. h_cost, 5. f_cost, 6. parent index_x
% 7. index_y
function index = util_lowest_f_index(openlist)

dim = size(openlist);
if dim(1) == 0
    index = -1;
else
    [~,index] = min(openlist(:,5));
end