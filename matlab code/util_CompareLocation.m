% determine whether two points are the same (integer)
% true if same, false if different
function isSame = util_CompareLocation(index1_x, index1_y, index2_x, index2_y)

if abs(index1_x-index2_x)+abs(index1_y-index2_y)<0.2
    isSame = true;
else
    isSame = false;
end