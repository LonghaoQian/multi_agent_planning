% heristic 
function h = util_Heuristic(x_start,y_start,x_destination, y_destination, enable_diagonal)

if(enable_diagonal)
    h = util_EulerDist(x_start,y_start,x_destination, y_destination);
else
    h = util_ManhDist(x_start,y_start,x_destination, y_destination);
end